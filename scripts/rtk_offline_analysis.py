#!/usr/bin/env python3
"""Offline first-pass analysis for BX100 raw logs.

This tool reports what is available in a raw recording. It does not call a
receiver solution an absolute truth and does not upgrade NMEA GGA quality 2
(differential) to RTK Fixed.
"""

import argparse
import csv
import math
from collections import Counter
from pathlib import Path


GGA_QUALITY = {
    0: "no_fix", 1: "single", 2: "differential", 3: "pps",
    4: "rtk_fixed", 5: "rtk_float", 6: "estimated", 7: "manual", 8: "simulation",
}


def fields(line):
    return line.split("*", 1)[0].split(",")


def coord(value, direction):
    if not value or len(direction) != 1:
        return None
    digits = 2 if direction in "NS" else 3 if direction in "EW" else 0
    if not digits:
        return None
    try:
        out = float(value[:digits]) + float(value[digits:]) / 60.0
        return -out if direction in "SW" else out
    except ValueError:
        return None


def nmea_checksum_valid(line):
    """Return True/False when checksum exists, otherwise None."""
    if not line.startswith("$"):
        return None
    try:
        body, supplied = line[1:].rsplit("*", 1)
    except ValueError:
        return None
    if len(supplied) != 2:
        return False
    try:
        expected = int(supplied, 16)
    except ValueError:
        return False
    actual = 0
    for byte in body.encode("ascii", errors="replace"):
        actual ^= byte
    return actual == expected


def ascii_crc32_valid(line):
    """Validate BX100/NovAtel-style ASCII CRC-32 after a leading '#'."""
    if not line.startswith("#"):
        return None
    try:
        body, supplied = line[1:].rsplit("*", 1)
    except ValueError:
        return False
    if len(supplied) != 8:
        return False
    try:
        expected = int(supplied, 16)
    except ValueError:
        return False
    crc = 0
    for byte in body.encode("ascii", errors="replace"):
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc == expected


def message_name(line):
    if line.startswith("$"):
        return line[1:].split(",", 1)[0]
    if line.startswith("#"):
        return line[1:].split(",", 1)[0].split(";", 1)[0]
    return "other"


def protocol_frames(line):
    """Split a physical line if a receiver starts another ASCII log early."""
    begin = 0
    for index, char in enumerate(line[1:], 1):
        if char in "$#":
            yield line[begin:index]
            begin = index
    yield line[begin:]


def parse_raw(path):
    """Yield validated GGA rows from raw bytes or timestamp-tab-line logs."""
    for number, raw in enumerate(path.read_text(errors="replace").splitlines(), 1):
        if "\t" in raw:
            stamp, line = raw.split("\t", 1)
            try:
                receipt_ns = int(stamp)
            except ValueError:
                receipt_ns = 0
        else:
            receipt_ns, line = 0, raw
        for frame in protocol_frames(line):
            if not (frame.startswith("$") and frame[3:6] == "GGA"):
                continue
            if nmea_checksum_valid(frame) is False:
                continue
            f = fields(frame)
            if len(f) < 10:
                continue
            try:
                lat, lon = coord(f[2], f[3]), coord(f[4], f[5])
                if lat is None or lon is None:
                    continue
                yield {
                    "line": number,
                    "receipt_ns": receipt_ns,
                    "utc": f[1],
                    "lat": lat,
                    "lon": lon,
                    "alt": float(f[9] or "nan"),
                    "quality": int(f[6] or 0),
                    "sats": int(f[7] or 0),
                    "hdop": float(f[8] or "nan"),
                    "age": float(f[13]) if len(f) > 13 and f[13] else math.nan,
                }
            except ValueError:
                continue


def raw_inventory(path):
    """Return message and NMEA checksum counts without parsing proprietary logs."""
    names, checksums, vendor_crcs = Counter(), Counter(), Counter()
    physical_lines, recovered_frames = 0, 0
    for raw in path.read_text(errors="replace").splitlines():
        physical_lines += 1
        line = raw.split("\t", 1)[-1] if "\t" in raw else raw
        frames = list(protocol_frames(line))
        recovered_frames += len(frames)
        for frame in frames:
            names[message_name(frame)] += 1
            valid = nmea_checksum_valid(frame)
            if valid is True:
                checksums["valid"] += 1
            elif valid is False:
                checksums["invalid"] += 1
            elif frame.startswith("$"):
                checksums["absent"] += 1
            vendor_valid = ascii_crc32_valid(frame)
            if vendor_valid is True:
                vendor_crcs["valid"] += 1
            elif vendor_valid is False:
                vendor_crcs["invalid"] += 1
    return names, checksums, vendor_crcs, physical_lines, recovered_frames


def ecef(lat, lon, h):
    a, e2 = 6378137.0, 6.6943799901413165e-3
    lat, lon = math.radians(lat), math.radians(lon)
    n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
    return ((n + h) * math.cos(lat) * math.cos(lon),
            (n + h) * math.cos(lat) * math.sin(lon),
            (n * (1 - e2) + h) * math.sin(lat))


def enu(row, origin):
    lat0, lon0, h0 = origin
    x0, y0, z0 = ecef(lat0, lon0, h0)
    x, y, z = ecef(row["lat"], row["lon"], row["alt"])
    lat0, lon0 = math.radians(lat0), math.radians(lon0)
    dx, dy, dz = x - x0, y - y0, z - z0
    east = -math.sin(lon0) * dx + math.cos(lon0) * dy
    north = (-math.sin(lat0) * math.cos(lon0) * dx
             - math.sin(lat0) * math.sin(lon0) * dy + math.cos(lat0) * dz)
    up = (math.cos(lat0) * math.cos(lon0) * dx
          + math.cos(lat0) * math.sin(lon0) * dy + math.sin(lat0) * dz)
    return east, north, up


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def static_summary(rows, start, end):
    """Return scatter relative to a window's own mean; no external truth implied."""
    window = rows[start:end]
    if not window:
        raise ValueError(f"empty static window {start}:{end}")
    origin = (
        sum(r["lat"] for r in window) / len(window),
        sum(r["lon"] for r in window) / len(window),
        sum(r["alt"] for r in window) / len(window),
    )
    points = [enu(row, origin) for row in window]
    east, north, up = zip(*points)
    radial = [math.hypot(e, n) for e, n in zip(east, north)]
    return {
        "samples": len(window),
        "utc_start": window[0]["utc"],
        "utc_end": window[-1]["utc"],
        "sigma_e_m": math.sqrt(sum(v * v for v in east) / len(east)),
        "sigma_n_m": math.sqrt(sum(v * v for v in north) / len(north)),
        "sigma_u_m": math.sqrt(sum(v * v for v in up) / len(up)),
        "horizontal_rms_m": math.sqrt(sum(v * v for v in radial) / len(radial)),
        "horizontal_p95_m": percentile(radial, 0.95),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path)
    parser.add_argument("--origin", nargs=3, type=float, metavar=("LAT", "LON", "H"),
                        help="WGS-84 ENU origin: LAT LON H")
    parser.add_argument("--csv", type=Path, help="write parsed GGA/ENU rows")
    parser.add_argument("--summary", type=Path, help="write summary text")
    args = parser.parse_args()

    rows = list(parse_raw(args.raw))
    names, checksums, vendor_crcs, physical_lines, recovered_frames = raw_inventory(args.raw)
    origin = tuple(args.origin) if args.origin else None
    states = Counter(GGA_QUALITY.get(r["quality"], "unknown") for r in rows)

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["line", "receipt_ns", "utc", "latitude", "longitude", "altitude",
                             "gga_quality", "solution_class", "satellites", "hdop",
                             "differential_age", "east_m", "north_m", "up_m"])
            for row in rows:
                values = [row[k] for k in ("line", "receipt_ns", "utc", "lat", "lon", "alt",
                                            "quality")]
                values += [GGA_QUALITY.get(row["quality"], "unknown"), row["sats"], row["hdop"], row["age"]]
                writer.writerow(values + list(enu(row, origin)) if origin else values)

    lines = [
        f"raw={args.raw}",
        f"physical_line_count={physical_lines}",
        f"recovered_protocol_frame_count={recovered_frames}",
        f"message_counts={dict(sorted(names.items()))}",
        f"nmea_checksum_counts={dict(checksums)}",
        f"ascii_crc32_counts={dict(vendor_crcs)}",
        f"gga_rows={len(rows)}",
        f"solution_class_counts={dict(states)}",
    ]
    if rows:
        ages = [r["age"] for r in rows if math.isfinite(r["age"])]
        lines += [f"first_utc={rows[0]['utc']}", f"last_utc={rows[-1]['utc']}"]
        lines += [f"satellites_min={min(r['sats'] for r in rows)}",
                  f"satellites_max={max(r['sats'] for r in rows)}",
                  f"hdop_min={min(r['hdop'] for r in rows):.3f}",
                  f"hdop_max={max(r['hdop'] for r in rows):.3f}"]
        if ages:
            lines += [f"differential_age_min_s={min(ages):.3f}",
                      f"differential_age_max_s={max(ages):.3f}"]
    else:
        lines.append("warning=no_valid_gga_rows")

    text = "\n".join(lines) + "\n"
    if args.summary:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(text)
    else:
        print(text, end="")


if __name__ == "__main__":
    main()

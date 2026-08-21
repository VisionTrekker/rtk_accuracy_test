#!/usr/bin/env python3
"""Offline first-pass analysis for BX100 raw logs.

The tool deliberately does not call a receiver output an absolute truth.  It
reports ENU coordinates, state durations, static scatter, and loop closure
when a reference origin is supplied.
"""
import argparse
import csv
import math
import re
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path


def fields(line):
    return line.split("*")[0].split(",")


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


def parse_raw(path):
    for number, raw in enumerate(path.read_text(errors="replace").splitlines(), 1):
        if "\t" in raw:
            stamp, line = raw.split("\t", 1)
            try:
                receipt_ns = int(stamp)
            except ValueError:
                receipt_ns = 0
        else:
            receipt_ns, line = 0, raw
        if line.startswith("$") and line[3:6] == "GGA":
            f = fields(line)
            if len(f) >= 10:
                lat, lon = coord(f[2], f[3]), coord(f[4], f[5])
                if lat is not None and lon is not None:
                    yield {"line": number, "receipt_ns": receipt_ns, "lat": lat,
                           "lon": lon, "alt": float(f[9] or "nan"),
                           "quality": int(f[6] or 0), "sats": int(f[7] or 0),
                           "hdop": float(f[8] or "nan"),
                           "age": float(f[13] or "nan") if len(f) > 13 and f[13] else math.nan}


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
    dl, dp = math.radians(lat0), math.radians(lon0)
    dx, dy, dz = x - x0, y - y0, z - z0
    east = -math.sin(dp) * dx + math.cos(dp) * dy
    north = (-math.sin(dl) * math.cos(dp) * dx - math.sin(dl) * math.sin(dp) * dy + math.cos(dl) * dz)
    up = math.cos(dl) * math.cos(dp) * dx + math.cos(dl) * math.sin(dp) * dy + math.sin(dl) * dz
    return east, north, up


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("raw", type=Path)
    parser.add_argument("--origin", nargs=3, type=float, metavar=("LAT", "LON", "H"), help="base station WGS-84 origin")
    parser.add_argument("--csv", type=Path, help="write parsed GGA/ENU rows")
    parser.add_argument("--summary", type=Path, help="write summary text")
    args = parser.parse_args()
    rows = list(parse_raw(args.raw))
    origin = tuple(args.origin) if args.origin else None
    state = Counter("fixed" if r["quality"] == 4 else "differential" if r["quality"] == 2 else "single_or_other" if r["quality"] else "no_fix" for r in rows)
    if args.csv:
        with args.csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["line", "receipt_ns", "latitude", "longitude", "altitude", "quality", "satellites", "hdop", "differential_age", "east_m", "north_m", "up_m"])
            for row in rows:
                writer.writerow([row[k] for k in ("line", "receipt_ns", "lat", "lon", "alt", "quality", "sats", "hdop", "age")] + list(enu(row, origin)) if origin else [row[k] for k in ("line", "receipt_ns", "lat", "lon", "alt", "quality", "sats", "hdop", "age")])
    lines = [f"raw={args.raw}", f"gga_rows={len(rows)}", f"quality_counts={dict(state)}"]
    if rows:
        ages = [r["age"] for r in rows if math.isfinite(r["age"])]
        lines.append(f"first_receipt_ns={rows[0]['receipt_ns']}")
        lines.append(f"last_receipt_ns={rows[-1]['receipt_ns']}")
        if ages:
            lines.append(f"differential_age_max_s={max(ages):.3f}")
    if not rows:
        lines.append("warning=no_valid_gga_rows")
    text = "\n".join(lines) + "\n"
    if args.summary:
        args.summary.write_text(text)
    else:
        print(text, end="")


if __name__ == "__main__":
    main()

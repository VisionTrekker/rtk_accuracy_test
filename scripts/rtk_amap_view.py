#!/usr/bin/env python3
"""Export validated GGA positions from a raw RTK log to a Gaode/AMap HTML viewer.

The generated HTML contains the track data, but loads the Gaode Web JS API only
when a browser has a valid API key.  By default the key is entered in the page
and stored only in that browser's localStorage; do not use --amap-key or
--security-js-code for HTML files that will be shared or committed.
"""

import argparse
import html
import json
import math
from collections import Counter
from pathlib import Path


PI = math.pi
A = 6378245.0
EE = 0.00669342162296594323

STATUS = {
    0: ("no_fix", "无定位", "#6b7280"),
    1: ("single", "单点定位", "#64748b"),
    2: ("differential", "差分定位", "#7c3aed"),
    3: ("pps", "PPS", "#475569"),
    4: ("rtk_fixed", "RTK Fixed", "#16a34a"),
    5: ("rtk_float", "RTK Float", "#ea580c"),
    6: ("estimated", "估计解", "#0f766e"),
    7: ("manual", "手动输入", "#a16207"),
    8: ("simulation", "仿真", "#2563eb"),
}
UNKNOWN_STATUS = ("unknown", "未知状态", "#334155")


def out_of_china(latitude, longitude):
    return longitude < 72.004 or longitude > 137.8347 or latitude < 0.8293 or latitude > 55.8271


def transform_latitude(x, y):
    value = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
    value += 0.2 * math.sqrt(abs(x))
    value += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    value += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    value += (160.0 * math.sin(y * PI / 12.0) + 320.0 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return value


def transform_longitude(x, y):
    value = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y
    value += 0.1 * math.sqrt(abs(x))
    value += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    value += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    value += (150.0 * math.sin(x * PI / 12.0) + 300.0 * math.sin(x * PI / 30.0)) * 2.0 / 3.0
    return value


def wgs84_to_gcj02(latitude, longitude):
    """Convert WGS-84 to the coordinate system expected by Gaode maps."""
    if out_of_china(latitude, longitude):
        return latitude, longitude

    delta_latitude = transform_latitude(longitude - 105.0, latitude - 35.0)
    delta_longitude = transform_longitude(longitude - 105.0, latitude - 35.0)
    rad_latitude = latitude / 180.0 * PI
    magic = 1.0 - EE * math.sin(rad_latitude) ** 2
    sqrt_magic = math.sqrt(magic)
    delta_latitude = (delta_latitude * 180.0) / ((A * (1.0 - EE)) / (magic * sqrt_magic) * PI)
    delta_longitude = (delta_longitude * 180.0) / (A / sqrt_magic * math.cos(rad_latitude) * PI)
    return latitude + delta_latitude, longitude + delta_longitude


def nmea_checksum_valid(sentence):
    """Return True only for a syntactically valid NMEA checksum."""
    if not sentence.startswith("$"):
        return False
    try:
        body, supplied = sentence[1:].rsplit("*", 1)
    except ValueError:
        return False
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


def protocol_frames(line):
    """Recover NMEA/vendor frames when a damaged physical line contains a new header."""
    begin = 0
    for index, character in enumerate(line[1:], 1):
        if character in "$#":
            yield line[begin:index]
            begin = index
    yield line[begin:]


def split_capture_line(raw):
    """Return optional host receipt timestamp and raw receiver text for one line."""
    if "\t" not in raw:
        return None, raw
    possible_timestamp, line = raw.split("\t", 1)
    try:
        return str(int(possible_timestamp)), line
    except ValueError:
        return None, raw


def nmea_coordinate(value, direction):
    if not value or len(direction) != 1:
        return None
    digits = 2 if direction in "NS" else 3 if direction in "EW" else 0
    if not digits:
        return None
    try:
        coordinate = float(value[:digits]) + float(value[digits:]) / 60.0
    except ValueError:
        return None
    if not math.isfinite(coordinate):
        return None
    return -coordinate if direction in "SW" else coordinate


def optional_float(value):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def parse_gga_points(raw_path):
    """Parse checksum-valid GGA samples and return them with export diagnostics."""
    points = []
    skipped = Counter()
    total_gga_frames = 0

    for physical_line, raw in enumerate(raw_path.read_text(errors="replace").splitlines(), 1):
        receipt_ns, line = split_capture_line(raw)
        for frame in protocol_frames(line):
            if not frame.startswith("$"):
                continue
            fields = frame.split("*", 1)[0].split(",")
            sentence_name = fields[0][1:] if fields else ""
            if not sentence_name.endswith("GGA"):
                continue
            total_gga_frames += 1
            if not nmea_checksum_valid(frame):
                skipped["invalid_or_absent_checksum"] += 1
                continue
            if len(fields) < 10:
                skipped["incomplete_gga"] += 1
                continue

            latitude = nmea_coordinate(fields[2], fields[3])
            longitude = nmea_coordinate(fields[4], fields[5])
            if latitude is None or longitude is None or not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
                skipped["invalid_coordinate"] += 1
                continue
            try:
                quality = int(fields[6] or 0)
            except ValueError:
                quality = -1
            status_key, status_label, color = STATUS.get(quality, UNKNOWN_STATUS)
            gcj_latitude, gcj_longitude = wgs84_to_gcj02(latitude, longitude)
            points.append({
                "index": len(points),
                "line": physical_line,
                "lat": gcj_latitude,
                "lon": gcj_longitude,
                "wgsLat": latitude,
                "wgsLon": longitude,
                "alt": optional_float(fields[9]),
                "utc": fields[1] or None,
                "receiptNs": receipt_ns,
                "quality": quality,
                "status": status_key,
                "statusLabel": status_label,
                "color": color,
            })
    return points, total_gga_frames, skipped


def build_segments(points):
    """Create contiguous same-state polyline segments without bridging state changes."""
    segments = []
    current = []
    current_status = None
    for point in points:
        if point["status"] != current_status:
            if current:
                segments.append({
                    "status": current_status,
                    "color": current[0]["color"],
                    "indices": [item["index"] for item in current],
                })
            current = [point]
            current_status = point["status"]
        else:
            current.append(point)
    if current:
        segments.append({
            "status": current_status,
            "color": current[0]["color"],
            "indices": [item["index"] for item in current],
        })
    return segments


def metadata(points):
    altitudes = [point["alt"] for point in points if point["alt"] is not None]
    status_counts = Counter(point["status"] for point in points)
    labels = {point["status"]: point["statusLabel"] for point in points}
    colors = {point["status"]: point["color"] for point in points}
    return {
        "pointCount": len(points),
        "statusCounts": [
            {"status": status, "label": labels[status], "color": colors[status], "count": count}
            for status, count in sorted(status_counts.items())
        ],
        "minLat": min(point["wgsLat"] for point in points),
        "maxLat": max(point["wgsLat"] for point in points),
        "minLon": min(point["wgsLon"] for point in points),
        "maxLon": max(point["wgsLon"] for point in points),
        "minAlt": min(altitudes) if altitudes else None,
        "maxAlt": max(altitudes) if altitudes else None,
    }


def script_json(value):
    """Serialize safely for an inline script, including user-provided titles."""
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")


def build_html(points, title, amap_key, security_js_code):
    if not points:
        raise ValueError("No checksum-valid GGA positions found; no HTML was written.")

    points_json = script_json(points)
    segments_json = script_json(build_segments(points))
    metadata_json = script_json(metadata(points))
    key_json = script_json(amap_key or "")
    security_json = script_json(security_js_code or "")
    title_json = script_json(title)
    safe_title = html.escape(title)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{safe_title}</title>
  <style>
    :root {{ color-scheme: light; }}
    html, body, #map {{ width: 100%; height: 100%; margin: 0; font-family: Arial, "Microsoft YaHei", sans-serif; }}
    .panel {{ position: absolute; z-index: 10; top: 12px; left: 12px; width: min(390px, calc(100vw - 24px)); max-height: calc(100vh - 24px); overflow: auto; box-sizing: border-box; background: rgba(255,255,255,.96); border: 1px solid #d8dee4; border-radius: 8px; box-shadow: 0 8px 24px rgba(0,0,0,.16); color: #1f2328; }}
    .panel-header {{ display: flex; align-items: center; justify-content: space-between; gap: 8px; padding: 10px 12px; border-bottom: 1px solid #d8dee4; font-size: 15px; font-weight: 700; }}
    .panel-body {{ padding: 10px 12px 12px; font-size: 13px; line-height: 1.55; }}
    .stats {{ display: grid; grid-template-columns: 1fr 1fr; gap: 8px 12px; }}
    .label {{ color: #57606a; }} .value {{ font-weight: 700; color: #0969da; word-break: break-word; }}
    .legend {{ margin-top: 10px; padding-top: 8px; border-top: 1px solid #d8dee4; }}
    .legend-row {{ display: flex; align-items: center; justify-content: space-between; gap: 8px; }}
    .legend-label {{ display: flex; align-items: center; gap: 6px; }}
    .swatch {{ width: 10px; height: 10px; border-radius: 50%; display: inline-block; }}
    .key-row {{ display: flex; gap: 6px; margin-top: 8px; }}
    input {{ min-width: 0; flex: 1; height: 32px; box-sizing: border-box; border: 1px solid #d0d7de; border-radius: 6px; padding: 0 8px; }}
    button {{ height: 32px; border: 1px solid #0969da; border-radius: 6px; padding: 0 10px; background: #0969da; color: #fff; cursor: pointer; font-weight: 700; }}
    .hint {{ margin-top: 8px; color: #57606a; font-size: 12px; }}
    .error {{ display: none; position: absolute; inset: 0; z-index: 8; align-items: center; justify-content: center; box-sizing: border-box; background: #f6f8fa; color: #24292f; padding: 24px; text-align: center; }}
    .error-inner {{ max-width: 560px; padding: 20px; border: 1px solid #d0d7de; border-radius: 8px; background: #fff; box-shadow: 0 8px 24px rgba(0,0,0,.08); }}
  </style>
</head>
<body>
  <div id="map"></div>
  <div id="error" class="error"><div class="error-inner"><h2>高德地图未加载</h2><p id="errorText"></p></div></div>
  <section class="panel" aria-label="轨迹信息">
    <div class="panel-header"><span id="title"></span><button id="fitBtn" type="button">居中</button></div>
    <div class="panel-body">
      <div class="stats">
        <div><span class="label">有效 GGA 点数</span><br><span class="value" id="pointCount"></span></div>
        <div><span class="label">高度范围</span><br><span class="value" id="altRange"></span></div>
        <div><span class="label">纬度范围（WGS‑84）</span><br><span class="value" id="latRange"></span></div>
        <div><span class="label">经度范围（WGS‑84）</span><br><span class="value" id="lonRange"></span></div>
      </div>
      <div class="legend"><div class="label">GGA 解状态（状态变化处断线）</div><div id="legendRows"></div></div>
      <div class="legend">
        <div class="label">高德 Web JS API Key</div>
        <div class="key-row"><input id="keyInput" placeholder="输入 Key 后加载地图" autocomplete="off"><button id="loadBtn" type="button">加载</button></div>
        <div class="hint">轨迹从 WGS‑84 转为 GCJ‑02 后绘制。未提供 Key 时，Key 仅保存到当前浏览器的 localStorage。</div>
      </div>
    </div>
  </section>
  <script>
    const TRACK_POINTS = {points_json};
    const TRACK_SEGMENTS = {segments_json};
    const META = {metadata_json};
    const EMBEDDED_KEY = {key_json};
    const SECURITY_JS_CODE = {security_json};
    const MAP_TITLE = {title_json};
    let map = null;
    let overlays = [];
    let mapRequested = false;

    const byId = id => document.getElementById(id);
    const escapeHtml = value => String(value == null ? "—" : value).replace(/[&<>"']/g, character => ({{"&":"&amp;","<":"&lt;",">":"&gt;","\\"":"&quot;","'":"&#39;"}}[character]));
    const formatCoordinate = value => value.toFixed(9);
    const formatAltitude = value => value == null ? "未记录" : `${{value.toFixed(3)}} m`;
    function formatHostReceipt(receiptNs) {{
      if (!receiptNs) return "未记录（原始字节流不含主机时间索引）";
      try {{ return new Date(Number(BigInt(receiptNs) / 1000000n)).toISOString(); }}
      catch (_) {{ return `${{receiptNs}} ns`; }}
    }}
    function showError(message) {{ byId("errorText").textContent = message; byId("error").style.display = "flex"; }}
    function hideError() {{ byId("error").style.display = "none"; }}
    function setStats() {{
      byId("title").textContent = MAP_TITLE;
      byId("pointCount").textContent = META.pointCount;
      byId("altRange").textContent = META.minAlt == null ? "无有效高程" : `${{META.minAlt.toFixed(3)}} ~ ${{META.maxAlt.toFixed(3)}} m`;
      byId("latRange").textContent = `${{META.minLat.toFixed(9)}} ~ ${{META.maxLat.toFixed(9)}}`;
      byId("lonRange").textContent = `${{META.minLon.toFixed(9)}} ~ ${{META.maxLon.toFixed(9)}}`;
      byId("legendRows").innerHTML = META.statusCounts.map(item => `<div class="legend-row"><span class="legend-label"><i class="swatch" style="background:${{item.color}}"></i>${{escapeHtml(item.label)}}</span><strong>${{item.count}}</strong></div>`).join("");
    }}
    function addEndpoint(point, label, color) {{
      const marker = new AMap.Marker({{
        position: [point.lon, point.lat], anchor: "bottom-center",
        content: `<div style="background:${{color}};color:#fff;padding:4px 8px;border-radius:6px;font-size:12px;font-weight:700;box-shadow:0 2px 8px rgba(0,0,0,.25);">${{label}}</div>`
      }});
      const info = new AMap.InfoWindow({{
        content: `<div style="font-size:13px;line-height:1.65;"><b>${{label}}</b><br>解状态：${{escapeHtml(point.statusLabel)}}<br>WGS‑84：${{formatCoordinate(point.wgsLat)}}, ${{formatCoordinate(point.wgsLon)}}<br>高程：${{formatAltitude(point.alt)}}<br>GGA UTC：${{escapeHtml(point.utc)}}<br>主机接收时间：${{escapeHtml(formatHostReceipt(point.receiptNs))}}<br>物理行：${{point.line}}</div>`,
        offset: new AMap.Pixel(0, -28)
      }});
      marker.on("click", () => info.open(map, marker.getPosition()));
      map.add(marker); overlays.push(marker);
    }}
    function initMap() {{
      if (!window.AMap) {{ showError("高德 Web JS API 未初始化。"); return; }}
      map = new AMap.Map("map", {{ zoom: 18, resizeEnable: true, viewMode: "2D", mapStyle: "amap://styles/normal" }});
      TRACK_SEGMENTS.forEach(segment => {{
        const segmentPoints = segment.indices.map(index => TRACK_POINTS[index]);
        let overlay;
        if (segmentPoints.length === 1) {{
          const point = segmentPoints[0];
          overlay = new AMap.CircleMarker({{
            center: [point.lon, point.lat], radius: 5, strokeColor: segment.color,
            strokeWeight: 2, fillColor: segment.color, fillOpacity: 0.9, zIndex: 20
          }});
        }} else {{
          overlay = new AMap.Polyline({{
            path: segmentPoints.map(point => [point.lon, point.lat]), strokeColor: segment.color,
            strokeOpacity: 0.95, strokeWeight: 5, lineJoin: "round", lineCap: "round", showDir: true
          }});
        }}
        map.add(overlay); overlays.push(overlay);
      }});
      addEndpoint(TRACK_POINTS[0], "起点", "#16a34a");
      addEndpoint(TRACK_POINTS[TRACK_POINTS.length - 1], "终点", "#dc2626");
      map.setFitView(overlays, false, [60, 60, 60, 60]);
      hideError();
    }}
    function loadAmap(key) {{
      if (!key) {{ showError("需要高德 Web JS API Key 才能加载底图。请在左上角输入后点击“加载”。"); return; }}
      if (mapRequested) return;
      mapRequested = true;
      try {{ localStorage.setItem("rtk_amap_key", key); }} catch (_) {{}}
      if (SECURITY_JS_CODE) window._AMapSecurityConfig = {{ securityJsCode: SECURITY_JS_CODE }};
      const script = document.createElement("script");
      script.src = `https://webapi.amap.com/maps?v=2.0&key=${{encodeURIComponent(key)}}`;
      script.onload = initMap;
      script.onerror = () => {{ mapRequested = false; showError("高德 JS API 加载失败，请检查网络、Key、安全码和域名白名单。"); }};
      document.head.appendChild(script);
    }}
    byId("fitBtn").addEventListener("click", () => {{ if (map && overlays.length) map.setFitView(overlays, false, [60, 60, 60, 60]); }});
    byId("loadBtn").addEventListener("click", () => loadAmap(byId("keyInput").value.trim()));
    setStats();
    let initialKey = EMBEDDED_KEY;
    if (!initialKey) {{ try {{ initialKey = localStorage.getItem("rtk_amap_key") || ""; }} catch (_) {{}} }}
    byId("keyInput").value = initialKey;
    if (initialKey) loadAmap(initialKey); else showError("请输入高德 Web JS API Key 后加载底图。轨迹统计已在左上角显示。");
  </script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw", type=Path, help="raw receiver log; timestamp-tab-line ROS logs are also accepted")
    parser.add_argument("output_html", type=Path, help="output self-contained HTML path")
    parser.add_argument("--title", help="page title (default: input filename + RTK trajectory)")
    parser.add_argument("--amap-key", default="", help="embed an AMap key in output HTML (do not share or commit it)")
    parser.add_argument("--security-js-code", default="", help="embed AMap securityJsCode in output HTML (do not share or commit it)")
    args = parser.parse_args()

    if not args.raw.is_file():
        parser.error(f"raw log does not exist or is not a file: {args.raw}")
    points, total_gga_frames, skipped = parse_gga_points(args.raw)
    if not points:
        parser.error("no checksum-valid GGA positions found; output was not written")

    title = args.title or f"{args.raw.name} RTK 轨迹"
    output_path = args.output_html.expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(build_html(points, title, args.amap_key, args.security_js_code), encoding="utf-8")
    states = Counter(point["status"] for point in points)
    print(f"wrote_html={output_path}")
    print(f"gga_frames_seen={total_gga_frames}")
    print(f"plotted_gga_points={len(points)}")
    print(f"status_counts={dict(sorted(states.items()))}")
    print(f"skipped_gga_counts={dict(sorted(skipped.items()))}")


if __name__ == "__main__":
    main()

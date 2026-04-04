#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import cv2


MODE_COLOR = {
    "attack": (0, 0, 255),
    "defense": (255, 0, 0),
    "sensitive": (0, 255, 0),
}

MODE_PRIORITY = {
    "attack": 100,
    "defense": 300,
    "sensitive": 200,
}


def parse_map_yaml(yaml_path: Path):
    resolution = None
    origin = None
    with yaml_path.open("r", encoding="utf-8") as file:
        for raw in file:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("resolution:"):
                resolution = float(line.split(":", 1)[1].strip())
            elif line.startswith("origin:"):
                raw_origin = line.split(":", 1)[1].strip()
                raw_origin = raw_origin.strip("[]")
                nums = [float(value.strip()) for value in raw_origin.split(",")]
                if len(nums) >= 2:
                    origin = (nums[0], nums[1])

    if resolution is None or origin is None:
        raise RuntimeError(f"无法从 {yaml_path} 读取 resolution/origin")
    return resolution, origin


def pixel_to_map(px: int, py: int, height: int, resolution: float, origin):
    map_x = origin[0] + (px + 0.5) * resolution
    map_y = origin[1] + (height - py - 0.5) * resolution
    return map_x, map_y


def rect_px_to_map(rect, height: int, resolution: float, origin):
    x1, y1, x2, y2 = rect
    map_x1, map_y1 = pixel_to_map(x1, y1, height, resolution, origin)
    map_x2, map_y2 = pixel_to_map(x2, y2, height, resolution, origin)
    return {
        "x_min": min(map_x1, map_x2),
        "x_max": max(map_x1, map_x2),
        "y_min": min(map_y1, map_y2),
        "y_max": max(map_y1, map_y2),
    }


def load_zones(csv_path: Path):
    zones = []
    if not csv_path.exists():
        return zones

    with csv_path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            mode = row.get("mode", "").strip().lower()
            if mode not in MODE_COLOR:
                continue
            zones.append(
                {
                    "mode": mode,
                    "x_min": float(row["x_min"]),
                    "x_max": float(row["x_max"]),
                    "y_min": float(row["y_min"]),
                    "y_max": float(row["y_max"]),
                    "priority": int(row.get("priority", MODE_PRIORITY[mode])),
                }
            )
    return zones


def save_zones(csv_path: Path, zones):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["mode", "x_min", "x_max", "y_min", "y_max", "priority"],
        )
        writer.writeheader()
        for zone in zones:
            writer.writerow(zone)


def map_rect_to_px(zone, height: int, resolution: float, origin):
    def map_to_px(mx, my):
        px = int(round((mx - origin[0]) / resolution))
        py = int(round(height - (my - origin[1]) / resolution))
        return px, py

    p1 = map_to_px(zone["x_min"], zone["y_min"])
    p2 = map_to_px(zone["x_max"], zone["y_max"])
    return p1[0], p1[1], p2[0], p2[1]


def draw_overlay(base_bgr, zones_px, current_mode: str):
    canvas = base_bgr.copy()
    for item in zones_px:
        x1, y1, x2, y2, mode = item
        color = MODE_COLOR[mode]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            canvas,
            mode,
            (min(x1, x2), min(y1, y2) - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    tips = [
        "1=attack(红) 2=defense(蓝) 3=sensitive(绿)",
        "鼠标左键拖拽画矩形, 右键撤销最后一个",
        f"当前模式: {current_mode}",
        "s=保存到CSV, q=退出",
    ]
    y = 24
    for line in tips:
        cv2.putText(canvas, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 2, cv2.LINE_AA)
        y += 24
    return canvas


def main():
    parser = argparse.ArgumentParser(description="在PGM地图上画模式区域并导出CSV")
    parser.add_argument("--pgm", required=True, help="pgm地图路径")
    parser.add_argument("--map-yaml", required=True, help="地图yaml路径(含resolution/origin)")
    parser.add_argument("--out", default="src/rm_communication/config/zone_modes.csv", help="输出CSV路径")
    args = parser.parse_args()

    pgm_path = Path(args.pgm)
    yaml_path = Path(args.map_yaml)
    out_path = Path(args.out)

    image = cv2.imread(str(pgm_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"读取地图失败: {pgm_path}")

    resolution, origin = parse_map_yaml(yaml_path)
    height, width = image.shape[:2]
    base_bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    zones = load_zones(out_path)
    zones_px = []
    for zone in zones:
        px_rect = map_rect_to_px(zone, height, resolution, origin)
        zones_px.append((*px_rect, zone["mode"]))

    current_mode = "attack"
    drawing = {"active": False, "start": (0, 0), "end": (0, 0)}

    def mouse_callback(event, x, y, _flags, _userdata):
        x_clamped = max(0, min(width - 1, x))
        y_clamped = max(0, min(height - 1, y))

        if event == cv2.EVENT_LBUTTONDOWN:
            drawing["active"] = True
            drawing["start"] = (x_clamped, y_clamped)
            drawing["end"] = (x_clamped, y_clamped)
        elif event == cv2.EVENT_MOUSEMOVE and drawing["active"]:
            drawing["end"] = (x_clamped, y_clamped)
        elif event == cv2.EVENT_LBUTTONUP and drawing["active"]:
            drawing["active"] = False
            drawing["end"] = (x_clamped, y_clamped)
            x1, y1 = drawing["start"]
            x2, y2 = drawing["end"]
            if abs(x2 - x1) > 2 and abs(y2 - y1) > 2:
                zones_px.append((x1, y1, x2, y2, current_mode))
        elif event == cv2.EVENT_RBUTTONDOWN:
            if zones_px:
                zones_px.pop()

    cv2.namedWindow("ZoneMapEditor", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("ZoneMapEditor", mouse_callback)

    while True:
        frame = draw_overlay(base_bgr, zones_px, current_mode)
        if drawing["active"]:
            x1, y1 = drawing["start"]
            x2, y2 = drawing["end"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), MODE_COLOR[current_mode], 2)

        cv2.imshow("ZoneMapEditor", frame)
        key = cv2.waitKey(20) & 0xFF

        if key == ord("1"):
            current_mode = "attack"
        elif key == ord("2"):
            current_mode = "defense"
        elif key == ord("3"):
            current_mode = "sensitive"
        elif key == ord("s"):
            map_zones = []
            for x1, y1, x2, y2, mode in zones_px:
                rect = rect_px_to_map((x1, y1, x2, y2), height, resolution, origin)
                rect["mode"] = mode
                rect["priority"] = MODE_PRIORITY[mode]
                map_zones.append(rect)
            save_zones(out_path, map_zones)
            print(f"已保存 {len(map_zones)} 个区域到: {out_path}")
        elif key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

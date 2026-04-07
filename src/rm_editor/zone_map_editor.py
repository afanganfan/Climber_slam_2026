#!/usr/bin/env python3
# python3 src/rm_editor/zone_map_editor.py
import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np


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
            points_raw = row.get("points", "").strip()
            if not points_raw:
                continue
            points = []
            for item in points_raw.split(";"):
                pair = item.strip()
                if not pair:
                    continue
                parts = pair.split(",")
                if len(parts) != 2:
                    continue
                points.append((float(parts[0]), float(parts[1])))
            if len(points) < 3:
                continue
            zones.append(
                {
                    "mode": mode,
                    "points": points,
                    "priority": int(row.get("priority", MODE_PRIORITY[mode])),
                }
            )
    return zones


def save_zones(csv_path: Path, zones):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=["mode", "points", "priority"],
        )
        writer.writeheader()
        for zone in zones:
            writer.writerow(zone)


def map_to_px(mx: float, my: float, height: int, resolution: float, origin):
    px = int(round((mx - origin[0]) / resolution - 0.5))
    py = int(round(height - (my - origin[1]) / resolution - 0.5))
    return px, py


def map_points_to_px(points, height: int, resolution: float, origin):
    return [map_to_px(mx, my, height, resolution, origin) for mx, my in points]


def polygon_centroid(points):
    area = 0.0
    cx = 0.0
    cy = 0.0
    count = len(points)
    for i in range(count):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % count]
        cross = x1 * y2 - x2 * y1
        area += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    if abs(area) < 1e-6:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        return sum(xs) / count, sum(ys) / count
    area *= 0.5
    cx /= (6.0 * area)
    cy /= (6.0 * area)
    return cx, cy


def rotate_points(points, angle_deg, center):
    angle = angle_deg * (3.141592653589793 / 180.0)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    cx, cy = center
    rotated = []
    for x, y in points:
        dx = x - cx
        dy = y - cy
        rx = dx * cos_a - dy * sin_a + cx
        ry = dx * sin_a + dy * cos_a + cy
        rotated.append((int(round(rx)), int(round(ry))))
    return rotated


def draw_overlay(base_bgr, zones_px, current_mode: str, selected_idx, draft_points):
    canvas = base_bgr.copy()
    for idx, item in enumerate(zones_px):
        points, mode = item
        color = MODE_COLOR[mode]
        thickness = 3 if idx == selected_idx else 2
        poly = np.array(points, dtype=np.int32)
        cv2.polylines(canvas, [poly], True, color, thickness)
        label_pos = tuple(poly[0])
        cv2.putText(
            canvas,
            mode,
            (label_pos[0], label_pos[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    if draft_points:
        poly = np.array(draft_points, dtype=np.int32)
        cv2.polylines(canvas, [poly], False, MODE_COLOR[current_mode], 2)
        for pt in poly:
            cv2.circle(canvas, pt, 2, MODE_COLOR[current_mode], -1)

    tips = [
        "1=attack(红) 2=defense(蓝) 3=sensitive(绿)",
        "左键: 选择/添加点  右键: 撤销  滚轮: 旋转选中多边形",
        "Enter/双击=闭合多边形  s=保存到CSV  q=退出",
        f"当前模式: {current_mode}",
    ]
    y = 24
    for line in tips:
        cv2.putText(canvas, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 2, cv2.LINE_AA)
        y += 24
    return canvas


def main():
    parser = argparse.ArgumentParser(description="在PGM地图上画模式区域并导出CSV")
    parser.add_argument("--pgm", default="assets/7v7 1.pgm", help="pgm地图路径")
    parser.add_argument("--map-yaml", default="assets/7v7.yaml", help="地图yaml路径(含resolution/origin)")
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
        points_px = map_points_to_px(zone["points"], height, resolution, origin)
        zones_px.append((points_px, zone["mode"]))

    current_mode = "attack"
    draft_points = []
    selected_idx = None

    def mouse_callback(event, x, y, _flags, _userdata):
        nonlocal selected_idx
        x_clamped = max(0, min(width - 1, x))
        y_clamped = max(0, min(height - 1, y))

        if event == cv2.EVENT_LBUTTONDOWN:
            if not draft_points:
                for idx in range(len(zones_px) - 1, -1, -1):
                    poly, _mode = zones_px[idx]
                    poly_np = np.array(poly, dtype=np.int32)
                    if cv2.pointPolygonTest(poly_np, (x_clamped, y_clamped), False) >= 0:
                        selected_idx = idx
                        return
                selected_idx = None
            draft_points.append((x_clamped, y_clamped))
        elif event == cv2.EVENT_LBUTTONDBLCLK:
            if len(draft_points) >= 3:
                zones_px.append((draft_points[:], current_mode))
                draft_points.clear()
        elif event == cv2.EVENT_RBUTTONDOWN:
            if draft_points:
                draft_points.pop()
            elif zones_px:
                zones_px.pop()
        elif event == cv2.EVENT_MOUSEWHEEL:
            if selected_idx is None:
                return
            delta = 5 if _flags > 0 else -5
            points, mode = zones_px[selected_idx]
            center = polygon_centroid(points)
            zones_px[selected_idx] = (rotate_points(points, delta, center), mode)

    cv2.namedWindow("ZoneMapEditor", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("ZoneMapEditor", mouse_callback)

    while True:
        frame = draw_overlay(base_bgr, zones_px, current_mode, selected_idx, draft_points)

        cv2.imshow("ZoneMapEditor", frame)
        key = cv2.waitKey(20) & 0xFF

        if key == ord("1"):
            current_mode = "attack"
        elif key == ord("2"):
            current_mode = "defense"
        elif key == ord("3"):
            current_mode = "sensitive"
        elif key in (10, 13):
            if len(draft_points) >= 3:
                zones_px.append((draft_points[:], current_mode))
                draft_points.clear()
        elif key == ord("s"):
            map_zones = []
            for points_px, mode in zones_px:
                points_map = [pixel_to_map(x, y, height, resolution, origin) for x, y in points_px]
                points_str = ";".join([f"{mx:.4f},{my:.4f}" for mx, my in points_map])
                map_zones.append(
                    {
                        "mode": mode,
                        "points": points_str,
                        "priority": MODE_PRIORITY[mode],
                    }
                )
            save_zones(out_path, map_zones)
            print(f"已保存 {len(map_zones)} 个区域到: {out_path}")
        elif key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

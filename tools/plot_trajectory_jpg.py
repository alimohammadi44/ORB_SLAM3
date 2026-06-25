#!/usr/bin/env python3
"""Generate a 2x2 JPG trajectory summary without requiring matplotlib."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def load_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            rows.append([float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])])
    if len(rows) < 2:
        raise SystemExit(f"Not enough trajectory poses in {path}")
    arr = np.asarray(rows, dtype=float)
    t = arr[:, 0]
    if np.nanmedian(np.abs(t)) > 1e12:
        t = t / 1e9
    return t, arr[:, 1:4]


def project_points(points: np.ndarray, rect: tuple[int, int, int, int], axes=(0, 1), equal=True) -> np.ndarray:
    x, y, w, h = rect
    xy = points[:, axes].astype(float)
    mn = xy.min(axis=0)
    mx = xy.max(axis=0)
    span = np.maximum(mx - mn, 1e-9)
    pad = 55
    sx = (w - 2 * pad) / span[0]
    sy = (h - 2 * pad) / span[1]
    if equal:
        s = min(sx, sy)
        sx = sy = s
    px = x + pad + (xy[:, 0] - mn[0]) * sx
    py = y + h - pad - (xy[:, 1] - mn[1]) * sy
    return np.column_stack([px, py]).astype(np.int32)


def project_3d(points: np.ndarray, rect: tuple[int, int, int, int]) -> np.ndarray:
    x, y, w, h = rect
    p = points - points.mean(axis=0)
    iso_x = p[:, 0] - 0.55 * p[:, 1]
    iso_y = p[:, 2] + 0.35 * p[:, 1]
    iso = np.column_stack([iso_x, iso_y])
    mn = iso.min(axis=0)
    mx = iso.max(axis=0)
    span = np.maximum(mx - mn, 1e-9)
    pad = 60
    s = min((w - 2 * pad) / span[0], (h - 2 * pad) / span[1])
    px = x + pad + (iso[:, 0] - mn[0]) * s
    py = y + h - pad - (iso[:, 1] - mn[1]) * s
    return np.column_stack([px, py]).astype(np.int32)


def draw_panel(img, rect, title):
    x, y, w, h = rect
    cv2.rectangle(img, (x, y), (x + w, y + h), (245, 245, 245), -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), (40, 40, 40), 1)
    cv2.putText(img, title, (x + 18, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (10, 10, 10), 2, cv2.LINE_AA)


def draw_grid(img, rect):
    x, y, w, h = rect
    for i in range(1, 5):
        xx = x + i * w // 5
        yy = y + i * h // 5
        cv2.line(img, (xx, y + 45), (xx, y + h - 35), (200, 200, 200), 1)
        cv2.line(img, (x + 45, yy), (x + w - 25, yy), (200, 200, 200), 1)


def draw_polyline(img, pts, color, thickness=2):
    if len(pts) > 1:
        cv2.polylines(img, [pts.reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA)
        cv2.circle(img, tuple(pts[0]), 6, (60, 180, 75), -1, cv2.LINE_AA)
        cv2.circle(img, tuple(pts[-1]), 6, (30, 30, 230), -1, cv2.LINE_AA)


def draw_time_plot(img, rect, t, values, title, ylabel):
    draw_panel(img, rect, title)
    draw_grid(img, rect)
    x, y, w, h = rect
    tt = t - t[0]
    xy = np.column_stack([tt, values])
    mn = xy.min(axis=0)
    mx = xy.max(axis=0)
    span = np.maximum(mx - mn, 1e-9)
    pad_l, pad_r, pad_t, pad_b = 55, 30, 50, 45
    px = x + pad_l + (xy[:, 0] - mn[0]) * ((w - pad_l - pad_r) / span[0])
    py = y + h - pad_b - (xy[:, 1] - mn[1]) * ((h - pad_t - pad_b) / span[1])
    pts = np.column_stack([px, py]).astype(np.int32)
    draw_polyline(img, pts, (30, 120, 230), 2)
    cv2.putText(img, "Time [s]", (x + w // 2 - 45, y + h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.putText(img, ylabel, (x + 8, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    t, xyz = load_trajectory(args.trajectory)
    canvas = np.full((900, 1200, 3), 255, np.uint8)
    panels = [
        (40, 35, 520, 365),
        (640, 35, 520, 365),
        (40, 485, 520, 365),
        (640, 485, 520, 365),
    ]

    draw_panel(canvas, panels[0], "3D flight path")
    pts3 = project_3d(xyz, panels[0])
    draw_polyline(canvas, pts3, (30, 120, 230), 2)
    cv2.putText(canvas, "ORB-SLAM3", (panels[0][0] + 20, panels[0][1] + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (30, 120, 230), 2, cv2.LINE_AA)

    draw_panel(canvas, panels[1], "Top view: X-Y")
    draw_grid(canvas, panels[1])
    pts_xy = project_points(xyz, panels[1], axes=(0, 1), equal=True)
    draw_polyline(canvas, pts_xy, (30, 120, 230), 2)
    cv2.putText(canvas, "X [m]", (panels[1][0] + 235, panels[1][1] + panels[1][3] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Y [m]", (panels[1][0] + 8, panels[1][1] + 200), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1, cv2.LINE_AA)

    draw_time_plot(canvas, panels[2], t, xyz[:, 2], "Altitude / Z over time", "Z [m]")

    draw_panel(canvas, panels[3], "Position error after alignment")
    cv2.putText(canvas, "Ground truth is required for this panel.", (panels[3][0] + 80, panels[3][1] + 170), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (40, 40, 40), 2, cv2.LINE_AA)
    cv2.putText(canvas, "Provide GT to compute aligned error.", (panels[3][0] + 98, panels[3][1] + 215), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 40, 40), 2, cv2.LINE_AA)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.out), canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()

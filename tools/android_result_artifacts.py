#!/usr/bin/env python3
"""Create plots and an annotated video for an Android ORB-SLAM3 run."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np


def load_camera_csv(path: Path) -> list[tuple[int, str]]:
    with path.open(newline="") as f:
        rows = csv.reader(f)
        next(rows)
        return [(int(row[0]), row[1]) for row in rows if row]


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


def interpolate_xyz(frame_times_s: np.ndarray, traj_times_s: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    out = np.zeros((len(frame_times_s), 3), dtype=float)
    for i in range(3):
        out[:, i] = np.interp(frame_times_s, traj_times_s, xyz[:, i])
    return out


def draw_plot_image(xyz: np.ndarray, current_index: int, size: int = 320) -> np.ndarray:
    image = np.full((size, size, 3), 255, np.uint8)
    xy = xyz[:, [0, 2]]
    finite = np.isfinite(xy).all(axis=1)
    if finite.sum() < 2:
        return image
    xy = xy[finite]
    margin = 28
    min_xy = xy.min(axis=0)
    max_xy = xy.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1e-6)
    scale = min((size - 2 * margin) / span[0], (size - 2 * margin) / span[1])
    pts = ((xy - min_xy) * scale + margin).astype(int)
    pts[:, 1] = size - pts[:, 1]

    if len(pts) > 1:
        cv2.polylines(image, [pts.reshape(-1, 1, 2)], False, (40, 120, 220), 2, cv2.LINE_AA)
    idx = min(max(current_index, 0), len(pts) - 1)
    cv2.circle(image, tuple(pts[idx]), 6, (0, 0, 230), -1, cv2.LINE_AA)
    cv2.rectangle(image, (0, 0), (size - 1, size - 1), (30, 30, 30), 1)
    cv2.putText(image, "Trajectory X-Z", (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 2, cv2.LINE_AA)
    return image


def save_static_plots(xyz: np.ndarray, out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for axes, name, labels in [
        ((0, 1), "trajectory_xy.png", ("x [m]", "y [m]")),
        ((0, 2), "trajectory_xz.png", ("x [m]", "z [m]")),
    ]:
        size = 900
        image = np.full((650, size, 3), 255, np.uint8)
        pts_src = xyz[:, axes]
        min_xy = pts_src.min(axis=0)
        max_xy = pts_src.max(axis=0)
        span = np.maximum(max_xy - min_xy, 1e-6)
        margin = 70
        scale = min((size - 2 * margin) / span[0], (650 - 2 * margin) / span[1])
        pts = ((pts_src - min_xy) * scale + margin).astype(int)
        pts[:, 1] = 650 - pts[:, 1]
        cv2.polylines(image, [pts.reshape(-1, 1, 2)], False, (40, 120, 220), 3, cv2.LINE_AA)
        cv2.circle(image, tuple(pts[0]), 8, (40, 180, 80), -1, cv2.LINE_AA)
        cv2.circle(image, tuple(pts[-1]), 8, (0, 0, 230), -1, cv2.LINE_AA)
        cv2.putText(image, name.replace(".png", ""), (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(image, labels[0], (size // 2 - 40, 625), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.putText(image, labels[1], (20, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
        cv2.imwrite(str(out_dir / name), image)


def make_video(dataset: Path, trajectory: Path, output: Path, fps: float) -> None:
    cam_rows = load_camera_csv(dataset / "mav0" / "cam0" / "data.csv")
    frame_times_s = np.asarray([ts / 1e9 for ts, _ in cam_rows], dtype=float)
    traj_t, traj_xyz = load_trajectory(trajectory)
    aligned_xyz = interpolate_xyz(frame_times_s, traj_t, traj_xyz)
    save_static_plots(traj_xyz, output.parent)

    first = cv2.imread(str(dataset / "mav0" / "cam0" / "data" / cam_rows[0][1]))
    if first is None:
        raise SystemExit("Cannot read first frame")
    height, width = first.shape[:2]
    writer = cv2.VideoWriter(
        str(output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise SystemExit(f"Cannot open video writer for {output}")

    total_distance = 0.0
    previous = aligned_xyz[0]
    for index, ((_, filename), xyz) in enumerate(zip(cam_rows, aligned_xyz)):
        frame = cv2.imread(str(dataset / "mav0" / "cam0" / "data" / filename))
        if frame is None:
            continue
        if index > 0 and np.isfinite(xyz).all() and np.isfinite(previous).all():
            total_distance += float(np.linalg.norm(xyz - previous))
        previous = xyz

        cv2.rectangle(frame, (0, 0), (width, 110), (0, 0, 0), -1)
        cv2.putText(frame, f"Frame {index + 1}/{len(cam_rows)}", (24, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"Position x={xyz[0]:.2f} y={xyz[1]:.2f} z={xyz[2]:.2f} m", (24, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"Path length approx {total_distance:.2f} m", (width - 420, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)

        plot = draw_plot_image(aligned_xyz, index, 320)
        y0 = height - plot.shape[0] - 20
        x0 = width - plot.shape[1] - 20
        frame[y0:y0 + plot.shape[0], x0:x0 + plot.shape[1]] = plot
        writer.write(frame)
    writer.release()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    video_path = args.out_dir / "annotated_android_slam.mp4"
    make_video(args.dataset.resolve(), args.trajectory.resolve(), video_path, args.fps)
    print(f"Saved {video_path}")
    print(f"Saved {args.out_dir / 'trajectory_xy.png'}")
    print(f"Saved {args.out_dir / 'trajectory_xz.png'}")


if __name__ == "__main__":
    main()

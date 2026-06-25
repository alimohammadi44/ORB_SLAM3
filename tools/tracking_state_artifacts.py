#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


STATE_COLORS = {
    "OK": (40, 160, 40),
    "OK_KLT": (40, 160, 40),
    "NOT_INITIALIZED": (40, 180, 220),
    "RECENTLY_LOST": (0, 140, 255),
    "LOST": (40, 40, 220),
    "NO_IMAGES_YET": (150, 150, 150),
    "SYSTEM_NOT_READY": (150, 150, 150),
}


def read_log(path):
    rows = []
    with Path(path).open(newline="") as f:
        for row in csv.DictReader(f):
            row["frame_index"] = int(row["frame_index"])
            row["time_s"] = float(row["time_s"])
            row["tracking_state"] = int(row["tracking_state"])
            row["failed"] = row["failed"] == "1"
            rows.append(row)
    if not rows:
        raise SystemExit(f"No tracking rows found in {path}")
    return rows


def frame_path(dataset, timestamp_ns):
    return Path(dataset) / "mav0" / "cam0" / "data" / f"{timestamp_ns}.png"


def draw_graph(rows, out_path):
    width, height = 1500, 620
    margin_l, margin_r, margin_t, margin_b = 90, 40, 45, 95
    img = np.full((height, width, 3), 255, np.uint8)

    t_min = rows[0]["time_s"]
    t_max = rows[-1]["time_s"]
    if t_max <= t_min:
        t_max = t_min + 1.0

    plot_x0, plot_x1 = margin_l, width - margin_r
    plot_y0, plot_y1 = margin_t, height - margin_b

    def tx(t):
        return int(plot_x0 + (t - t_min) / (t_max - t_min) * (plot_x1 - plot_x0))

    levels = [
        ("OK", 0),
        ("OK_KLT", 0),
        ("NOT_INITIALIZED", 1),
        ("RECENTLY_LOST", 2),
        ("LOST", 3),
        ("NO_IMAGES_YET", 1),
        ("SYSTEM_NOT_READY", 1),
    ]
    level_map = {name: level for name, level in levels}
    y_labels = ["OK", "INIT", "RECENTLY_LOST", "LOST"]

    def ty(level):
        return int(plot_y1 - level / 3.0 * (plot_y1 - plot_y0))

    for i, label in enumerate(y_labels):
        y = ty(i)
        cv2.line(img, (plot_x0, y), (plot_x1, y), (220, 220, 220), 1)
        cv2.putText(img, label, (12, y + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 40, 40), 1, cv2.LINE_AA)

    for sec in np.linspace(t_min, t_max, 11):
        x = tx(float(sec))
        cv2.line(img, (x, plot_y0), (x, plot_y1), (235, 235, 235), 1)
        cv2.putText(img, f"{sec:.1f}", (x - 18, plot_y1 + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1, cv2.LINE_AA)

    failed_segments = []
    start = None
    prev_t = rows[0]["time_s"]
    for row in rows:
        if row["failed"] and start is None:
            start = row["time_s"]
        if not row["failed"] and start is not None:
            failed_segments.append((start, prev_t))
            start = None
        prev_t = row["time_s"]
    if start is not None:
        failed_segments.append((start, rows[-1]["time_s"]))

    for a, b in failed_segments:
        cv2.rectangle(img, (tx(a), plot_y0), (max(tx(b), tx(a) + 2), plot_y1), (225, 235, 255), -1)

    pts = []
    for row in rows:
        level = level_map.get(row["state_name"], 1)
        pts.append((tx(row["time_s"]), ty(level)))

    for p0, p1 in zip(pts, pts[1:]):
        color = (40, 40, 220) if p1[1] >= ty(2) else (40, 160, 40)
        cv2.line(img, p0, p1, color, 2)

    cv2.rectangle(img, (plot_x0, plot_y0), (plot_x1, plot_y1), (60, 60, 60), 1)
    cv2.putText(img, "Tracking state over time", (plot_x0, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(img, "Time [s]", ((plot_x0 + plot_x1) // 2 - 35, height - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 40), 1, cv2.LINE_AA)

    failed_frames = sum(1 for r in rows if r["failed"])
    cv2.putText(img, f"Failed frames: {failed_frames}/{len(rows)}", (plot_x1 - 260, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (40, 40, 180), 2, cv2.LINE_AA)

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img, [int(cv2.IMWRITE_JPEG_QUALITY), 94])


def draw_lamp_video(rows, dataset, out_path, fps):
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    writer = None
    for row in rows:
        img_path = frame_path(dataset, row["timestamp_ns"])
        frame = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        h, w = frame.shape[:2]
        if writer is None:
            writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            if not writer.isOpened():
                raise SystemExit(f"Could not open video writer for {out_path}")

        state = row["state_name"]
        color = STATE_COLORS.get(state, (0, 0, 255))
        if row["failed"]:
            color = (0, 0, 255)
        center = (w - 72, 72)
        cv2.circle(frame, center, 34, (20, 20, 20), -1, cv2.LINE_AA)
        cv2.circle(frame, center, 27, color, -1, cv2.LINE_AA)
        cv2.putText(frame, state, (25, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
        cv2.putText(frame, f"t={row['time_s']:.2f}s frame={row['frame_index']}", (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA)
        writer.write(frame)

    if writer is None:
        raise SystemExit("No frames were written. Check dataset path and timestamp filenames.")
    writer.release()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--graph", required=True)
    parser.add_argument("--video", required=True)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args()

    rows = read_log(args.log)
    draw_graph(rows, args.graph)
    draw_lamp_video(rows, args.dataset, args.video, args.fps)
    print(f"Saved {args.graph}")
    print(f"Saved {args.video}")


if __name__ == "__main__":
    main()

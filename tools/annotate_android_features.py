#!/usr/bin/env python3
"""Create an annotated camera video with ORB feature points."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2


def load_camera_rows(dataset: Path) -> list[tuple[str, int]]:
    with (dataset / "mav0" / "cam0" / "data.csv").open(newline="") as f:
        rows = csv.reader(f)
        next(rows)
        return [(row[1], int(row[0])) for row in rows if row]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--features", type=int, default=1500)
    args = parser.parse_args()

    rows = load_camera_rows(args.dataset)
    image_dir = args.dataset / "mav0" / "cam0" / "data"
    first = cv2.imread(str(image_dir / rows[0][0]))
    if first is None:
        raise SystemExit("Cannot read first frame")

    h, w = first.shape[:2]
    args.out.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.out),
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (w, h),
    )
    if not writer.isOpened():
        raise SystemExit(f"Cannot create {args.out}")

    orb = cv2.ORB_create(nfeatures=args.features)
    for idx, (filename, timestamp_ns) in enumerate(rows):
        frame = cv2.imread(str(image_dir / filename))
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints = orb.detect(gray, None)
        annotated = cv2.drawKeypoints(
            frame,
            keypoints,
            None,
            color=(0, 255, 0),
            flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS,
        )
        cv2.rectangle(annotated, (0, 0), (w, 86), (0, 0, 0), -1)
        cv2.putText(
            annotated,
            f"Frame {idx + 1}/{len(rows)}  ORB features: {len(keypoints)}",
            (24, 34),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"timestamp_ns: {timestamp_ns}",
            (24, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        writer.write(annotated)

    writer.release()
    print(f"Saved {args.out}")


if __name__ == "__main__":
    main()

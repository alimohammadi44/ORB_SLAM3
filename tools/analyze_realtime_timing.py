#!/usr/bin/env python3
import argparse
import csv
import shutil
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def load_csv(path):
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)

    if not rows:
        raise RuntimeError("CSV is empty.")

    data = {}
    for k in rows[0].keys():
        vals = []
        for r in rows:
            try:
                vals.append(float(r[k]))
            except Exception:
                vals.append(np.nan)
        data[k] = np.array(vals, dtype=float)

    return data, rows


def summarize(x):
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    return {
        "mean": float(np.mean(x)),
        "median": float(np.median(x)),
        "p90": float(np.percentile(x, 90)),
        "p95": float(np.percentile(x, 95)),
        "max": float(np.max(x)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    csv_path = Path(args.csv).expanduser()
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    data, rows = load_csv(csv_path)

    stages = [
        "image_read_ms",
        "imu_batch_ms",
        "track_ms",
        "total_compute_ms",
    ]

    frame = data["frame_id"]
    deadline_miss_rate = 100.0 * np.nanmean(data["deadline_miss"])
    mean_budget = np.nanmean(data["realtime_budget_ms"])

    lines = []
    lines.append("ORB-SLAM3 Mono-Inertial Real-Time Timing Summary")
    lines.append("=" * 60)
    lines.append(f"Input CSV: {csv_path}")
    lines.append(f"Frames analyzed: {len(rows)}")
    lines.append(f"Mean real-time budget: {mean_budget:.3f} ms")
    lines.append(f"Deadline miss rate: {deadline_miss_rate:.2f} %")
    lines.append("")

    for stage in stages:
        stats = summarize(data[stage])
        lines.append(stage)
        lines.append(f"  mean   : {stats['mean']:.3f} ms")
        lines.append(f"  median : {stats['median']:.3f} ms")
        lines.append(f"  p90    : {stats['p90']:.3f} ms")
        lines.append(f"  p95    : {stats['p95']:.3f} ms")
        lines.append(f"  max    : {stats['max']:.3f} ms")
        lines.append("")

    rtf = summarize(data["realtime_factor"])
    lines.append("Real-time factor = total_compute_ms / realtime_budget_ms")
    lines.append(f"  mean   : {rtf['mean']:.3f}")
    lines.append(f"  median : {rtf['median']:.3f}")
    lines.append(f"  p90    : {rtf['p90']:.3f}")
    lines.append(f"  p95    : {rtf['p95']:.3f}")
    lines.append(f"  max    : {rtf['max']:.3f}")
    lines.append("")

    if rtf["p95"] < 1.0 and deadline_miss_rate < 5.0:
        lines.append("Conclusion: PASS. The run is real-time at the dataset camera rate.")
    else:
        lines.append("Conclusion: WARNING. The run is not consistently real-time.")

    summary_path = out_dir / "timing_summary.txt"
    summary_path.write_text("\n".join(lines))
    print(summary_path.read_text())

    plt.figure(figsize=(12, 6))
    plt.plot(frame, data["image_read_ms"], label="Image read")
    plt.plot(frame, data["imu_batch_ms"], label="IMU batching")
    plt.plot(frame, data["track_ms"], label="ORB-SLAM3 tracking")
    plt.plot(frame, data["total_compute_ms"], label="Total compute")
    plt.plot(frame, data["realtime_budget_ms"], linestyle="--", label="Real-time budget")
    plt.xlabel("Frame index")
    plt.ylabel("Time [ms]")
    plt.title("Per-frame computation time versus real-time budget")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "timing_per_frame.png", dpi=200)
    plt.close()

    plt.figure(figsize=(9, 6))
    plt.boxplot(
        [data[s][np.isfinite(data[s])] for s in stages],
        labels=["Image read", "IMU batch", "Track", "Total"]
    )
    plt.ylabel("Time [ms]")
    plt.title("Distribution of computation time by processing stage")
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(out_dir / "timing_stage_boxplot.png", dpi=200)
    plt.close()

    margin = data["realtime_budget_ms"] - data["total_compute_ms"]

    plt.figure(figsize=(12, 5))
    plt.plot(frame, margin)
    plt.axhline(0, linestyle="--")
    plt.xlabel("Frame index")
    plt.ylabel("Real-time margin [ms]")
    plt.title("Real-time margin per frame")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(out_dir / "realtime_margin.png", dpi=200)
    plt.close()

    shutil.copy2(csv_path, out_dir / "timing_cleaned.csv")

    print("")
    print("Saved analysis files:")
    print(out_dir / "timing_summary.txt")
    print(out_dir / "timing_per_frame.png")
    print(out_dir / "timing_stage_boxplot.png")
    print(out_dir / "realtime_margin.png")
    print(out_dir / "timing_cleaned.csv")


if __name__ == "__main__":
    main()

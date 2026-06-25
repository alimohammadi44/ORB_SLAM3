#!/usr/bin/env python3
"""Export the same four-panel trajectory plot used by trajectory_gui.py."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np

from trajectory_gui import (
    compute_error,
    interpolate_gt_to_est_times,
    load_ground_truth,
    load_txt_trajectory,
    umeyama_align,
)


def save_aligned_csv(path: Path, t, gt, est_aligned, est_raw, err) -> None:
    arr = np.column_stack([t, gt, est_aligned, est_raw, err])
    header = (
        "timestamp,"
        "gt_x,gt_y,gt_z,"
        "est_aligned_x,est_aligned_y,est_aligned_z,"
        "est_raw_x,est_raw_y,est_raw_z,"
        "error_m"
    )
    np.savetxt(path, arr, delimiter=",", header=header, comments="")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--est", required=True, type=Path, help="ORB-SLAM3 CameraTrajectory.txt")
    parser.add_argument("--gt", required=True, type=Path, help="Ground truth txt or bag")
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--no-scale", action="store_true", help="Disable scale alignment")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)

    t_est, est_xyz = load_txt_trajectory(args.est)
    t_gt, gt_xyz = load_ground_truth(args.gt)
    t_match, est_mask, gt_match = interpolate_gt_to_est_times(t_est, t_gt, gt_xyz)
    est_match = est_xyz[est_mask]
    est_aligned, scale, _, _ = umeyama_align(est_match, gt_match, with_scale=not args.no_scale)
    err = compute_error(est_aligned, gt_match)

    fig = Figure(figsize=(11, 8), dpi=120)
    FigureCanvasAgg(fig)

    ax3d = fig.add_subplot(221, projection="3d")
    ax3d.plot(gt_match[:, 0], gt_match[:, 1], gt_match[:, 2], label="Ground truth")
    ax3d.plot(est_aligned[:, 0], est_aligned[:, 1], est_aligned[:, 2], label="ORB-SLAM3 aligned")
    ax3d.set_title("3D flight path")
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    ax3d.legend()

    ax_xy = fig.add_subplot(222)
    ax_xy.plot(gt_match[:, 0], gt_match[:, 1], label="Ground truth")
    ax_xy.plot(est_aligned[:, 0], est_aligned[:, 1], label="ORB-SLAM3 aligned")
    ax_xy.set_title("Top view: X-Y")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True)
    ax_xy.legend()

    tt = t_match - t_match[0]
    ax_z = fig.add_subplot(223)
    ax_z.plot(tt, gt_match[:, 2], label="Ground truth Z")
    ax_z.plot(tt, est_aligned[:, 2], label="ORB-SLAM3 Z aligned")
    ax_z.set_title("Altitude / Z over time")
    ax_z.set_xlabel("Time [s]")
    ax_z.set_ylabel("Z [m]")
    ax_z.grid(True)
    ax_z.legend()

    ax_e = fig.add_subplot(224)
    ax_e.plot(tt, err["errors"])
    ax_e.set_title("Position error after alignment")
    ax_e.set_xlabel("Time [s]")
    ax_e.set_ylabel("Error [m]")
    ax_e.grid(True)

    fig.tight_layout()
    fig.savefig(args.out_dir / "trajectory_gui_plot.png")

    save_aligned_csv(
        args.out_dir / "aligned_trajectory_comparison.csv",
        t_match,
        gt_match,
        est_aligned,
        est_match,
        err["errors"],
    )

    summary = [
        "Trajectory GUI Plot Summary",
        "=" * 60,
        f"Estimate: {args.est}",
        f"Ground truth: {args.gt}",
        f"Estimate poses: {len(t_est)}",
        f"GT poses: {len(t_gt)}",
        f"Matched poses: {len(t_match)}",
        f"Scale alignment: {not args.no_scale}",
        f"Scale: {scale:.6f}",
        f"RMSE: {err['rmse']:.6f} m",
        f"Mean: {err['mean']:.6f} m",
        f"Median: {err['median']:.6f} m",
        f"Max: {err['max']:.6f} m",
    ]
    (args.out_dir / "trajectory_gui_summary.txt").write_text("\n".join(summary) + "\n")
    print("\n".join(summary))
    print(f"Saved {args.out_dir / 'trajectory_gui_plot.png'}")


if __name__ == "__main__":
    main()

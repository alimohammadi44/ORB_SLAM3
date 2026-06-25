#!/usr/bin/env python3
import argparse
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox

import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk


GT_TOPIC = "/leica/pose/relative"


def stamp_to_ns(stamp):
    if hasattr(stamp, "sec"):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if hasattr(stamp, "secs"):
        return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
    raise RuntimeError("Unknown ROS stamp format")


def msg_time_ns(msg, fallback_ns):
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        return stamp_to_ns(msg.header.stamp)
    return int(fallback_ns)


def load_txt_trajectory(path):
    path = Path(path).expanduser()
    rows = []

    with path.open("r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue

            parts = line.split()
            if len(parts) < 4:
                continue

            try:
                vals = [float(x) for x in parts[:4]]
            except ValueError:
                continue

            rows.append(vals)

    if len(rows) < 2:
        raise RuntimeError(f"Not enough trajectory rows in: {path}")

    arr = np.asarray(rows, dtype=float)
    t = arr[:, 0]

    # ORB-SLAM3 EuRoC-style output may save timestamps in nanoseconds.
    # Leica ground truth is in seconds. Convert ns -> s when needed.
    if np.nanmedian(np.abs(t)) > 1e12:
        t = t / 1e9

    xyz = arr[:, 1:4]

    order = np.argsort(t)
    return t[order], xyz[order]


def load_gt_from_bag(path, topic=GT_TOPIC):
    from rosbags.highlevel import AnyReader

    path = Path(path).expanduser()
    rows = []

    with AnyReader([path]) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise RuntimeError(f"Ground-truth topic not found: {topic}")

        for conn, timestamp, rawdata in reader.messages(connections=conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            t = msg_time_ns(msg, timestamp) / 1e9
            p = msg.pose.position
            rows.append([t, p.x, p.y, p.z])

    if len(rows) < 2:
        raise RuntimeError(f"Not enough ground-truth poses in bag: {path}")

    arr = np.asarray(rows, dtype=float)
    order = np.argsort(arr[:, 0])
    return arr[order, 0], arr[order, 1:4]


def load_ground_truth(path):
    path = Path(path).expanduser()
    if path.suffix.lower() == ".bag":
        return load_gt_from_bag(path)
    return load_txt_trajectory(path)


def interpolate_gt_to_est_times(t_est, t_gt, gt_xyz):
    """
    Match estimated trajectory and ground truth.

    Method 1: absolute timestamps.
    Method 2: relative timestamps from start.
    Method 3: normalized progress matching, used only for visualization
              when timestamps are not compatible.
    """
    t_est = np.asarray(t_est, dtype=float)
    t_gt = np.asarray(t_gt, dtype=float)

    # Method 1: absolute timestamp overlap
    t0 = max(t_est.min(), t_gt.min())
    t1 = min(t_est.max(), t_gt.max())
    mask = (t_est >= t0) & (t_est <= t1)

    if mask.sum() >= 5:
        t_match = t_est[mask]
        gt_interp = np.column_stack([
            np.interp(t_match, t_gt, gt_xyz[:, 0]),
            np.interp(t_match, t_gt, gt_xyz[:, 1]),
            np.interp(t_match, t_gt, gt_xyz[:, 2]),
        ])
        return t_match, mask, gt_interp

    # Method 2: relative timestamp overlap
    t_est_rel = t_est - t_est.min()
    t_gt_rel = t_gt - t_gt.min()

    t0 = max(t_est_rel.min(), t_gt_rel.min())
    t1 = min(t_est_rel.max(), t_gt_rel.max())
    mask = (t_est_rel >= t0) & (t_est_rel <= t1)

    if mask.sum() >= 5:
        t_match_rel = t_est_rel[mask]
        gt_interp = np.column_stack([
            np.interp(t_match_rel, t_gt_rel, gt_xyz[:, 0]),
            np.interp(t_match_rel, t_gt_rel, gt_xyz[:, 1]),
            np.interp(t_match_rel, t_gt_rel, gt_xyz[:, 2]),
        ])
        return t_match_rel, mask, gt_interp

    # Method 3: normalized progress fallback.
    # This ignores timestamps and compares trajectory shape.
    n_est = len(t_est)
    n_gt = len(t_gt)

    if n_est < 5 or n_gt < 5:
        raise RuntimeError(
            f"Too few poses. Estimate poses={n_est}, GT poses={n_gt}. "
            "Check CameraTrajectory.txt and groundtruth_leica_60s.txt."
        )

    prog_gt = np.linspace(0.0, 1.0, n_gt)
    prog_est = np.linspace(0.0, 1.0, n_est)

    gt_interp = np.column_stack([
        np.interp(prog_est, prog_gt, gt_xyz[:, 0]),
        np.interp(prog_est, prog_gt, gt_xyz[:, 1]),
        np.interp(prog_est, prog_gt, gt_xyz[:, 2]),
    ])

    mask = np.ones(n_est, dtype=bool)
    t_match = prog_est

    return t_match, mask, gt_interp


def umeyama_align(src, dst, with_scale=True):
    """
    Align src trajectory to dst trajectory.
    Returns aligned_src, scale, rotation, translation.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)

    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)

    X = src - mu_src
    Y = dst - mu_dst

    cov = (Y.T @ X) / len(src)
    U, D, Vt = np.linalg.svd(cov)

    S = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        S[2, 2] = -1

    R = U @ S @ Vt

    if with_scale:
        var_src = np.mean(np.sum(X * X, axis=1))
        scale = np.trace(np.diag(D) @ S) / max(var_src, 1e-12)
    else:
        scale = 1.0

    trans = mu_dst - scale * R @ mu_src
    aligned = (scale * (R @ src.T)).T + trans

    return aligned, scale, R, trans


def compute_error(a, b):
    e = np.linalg.norm(a - b, axis=1)
    return {
        "rmse": float(np.sqrt(np.mean(e ** 2))),
        "mean": float(np.mean(e)),
        "median": float(np.median(e)),
        "max": float(np.max(e)),
        "errors": e,
    }


class TrajectoryGUI:
    def __init__(self, root, est_default, gt_default):
        self.root = root
        self.root.title("ORB-SLAM3 UAV Navigation vs Ground Truth")

        self.last_data = None

        control = tk.Frame(root)
        control.pack(side=tk.TOP, fill=tk.X, padx=8, pady=6)

        tk.Label(control, text="Estimated ORB-SLAM3 trajectory:").grid(row=0, column=0, sticky="w")
        self.est_entry = tk.Entry(control, width=90)
        self.est_entry.grid(row=0, column=1, padx=4)
        self.est_entry.insert(0, str(est_default))
        tk.Button(control, text="Browse", command=self.browse_est).grid(row=0, column=2)

        tk.Label(control, text="Ground truth txt or bag:").grid(row=1, column=0, sticky="w")
        self.gt_entry = tk.Entry(control, width=90)
        self.gt_entry.grid(row=1, column=1, padx=4)
        self.gt_entry.insert(0, str(gt_default))
        tk.Button(control, text="Browse", command=self.browse_gt).grid(row=1, column=2)

        self.scale_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            control,
            text="Allow scale alignment for visualization",
            variable=self.scale_var
        ).grid(row=2, column=1, sticky="w")

        tk.Button(control, text="Plot trajectories", command=self.plot).grid(row=2, column=0, sticky="w")
        tk.Button(control, text="Save aligned CSV", command=self.save_csv).grid(row=2, column=2, sticky="e")

        self.status = tk.Label(root, text="Ready", anchor="w")
        self.status.pack(side=tk.BOTTOM, fill=tk.X)

        self.fig = Figure(figsize=(11, 8), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        toolbar = NavigationToolbar2Tk(self.canvas, root)
        toolbar.update()

    def browse_est(self):
        p = filedialog.askopenfilename(title="Select CameraTrajectory.txt")
        if p:
            self.est_entry.delete(0, tk.END)
            self.est_entry.insert(0, p)

    def browse_gt(self):
        p = filedialog.askopenfilename(title="Select ground truth txt or eee_03.bag")
        if p:
            self.gt_entry.delete(0, tk.END)
            self.gt_entry.insert(0, p)

    def plot(self):
        try:
            est_path = Path(self.est_entry.get()).expanduser()
            gt_path = Path(self.gt_entry.get()).expanduser()

            self.status.config(text="Loading trajectories...")
            self.root.update_idletasks()

            t_est, est_xyz = load_txt_trajectory(est_path)
            t_gt, gt_xyz = load_ground_truth(gt_path)

            t_match, est_mask, gt_match = interpolate_gt_to_est_times(t_est, t_gt, gt_xyz)
            est_match = est_xyz[est_mask]

            est_aligned, scale, R, trans = umeyama_align(
                est_match,
                gt_match,
                with_scale=self.scale_var.get()
            )

            err = compute_error(est_aligned, gt_match)

            self.last_data = {
                "t": t_match,
                "gt": gt_match,
                "est_raw": est_match,
                "est_aligned": est_aligned,
                "err": err["errors"],
            }

            self.fig.clear()

            ax3d = self.fig.add_subplot(221, projection="3d")
            ax3d.plot(gt_match[:, 0], gt_match[:, 1], gt_match[:, 2], label="Ground truth")
            ax3d.plot(est_aligned[:, 0], est_aligned[:, 1], est_aligned[:, 2], label="ORB-SLAM3 aligned")
            ax3d.set_title("3D flight path")
            ax3d.set_xlabel("X [m]")
            ax3d.set_ylabel("Y [m]")
            ax3d.set_zlabel("Z [m]")
            ax3d.legend()

            ax_xy = self.fig.add_subplot(222)
            ax_xy.plot(gt_match[:, 0], gt_match[:, 1], label="Ground truth")
            ax_xy.plot(est_aligned[:, 0], est_aligned[:, 1], label="ORB-SLAM3 aligned")
            ax_xy.set_title("Top view: X-Y")
            ax_xy.set_xlabel("X [m]")
            ax_xy.set_ylabel("Y [m]")
            ax_xy.axis("equal")
            ax_xy.grid(True)
            ax_xy.legend()

            ax_z = self.fig.add_subplot(223)
            tt = t_match - t_match[0]
            ax_z.plot(tt, gt_match[:, 2], label="Ground truth Z")
            ax_z.plot(tt, est_aligned[:, 2], label="ORB-SLAM3 Z aligned")
            ax_z.set_title("Altitude / Z over time")
            ax_z.set_xlabel("Time [s]")
            ax_z.set_ylabel("Z [m]")
            ax_z.grid(True)
            ax_z.legend()

            ax_e = self.fig.add_subplot(224)
            ax_e.plot(tt, err["errors"])
            ax_e.set_title("Position error after alignment")
            ax_e.set_xlabel("Time [s]")
            ax_e.set_ylabel("Error [m]")
            ax_e.grid(True)

            self.fig.tight_layout()
            self.canvas.draw()

            msg = (
                f"Loaded estimate: {len(t_est)} poses | GT: {len(t_gt)} poses | "
                f"matched: {len(t_match)} | scale: {scale:.4f} | "
                f"RMSE: {err['rmse']:.3f} m | mean: {err['mean']:.3f} m | "
                f"median: {err['median']:.3f} m | max: {err['max']:.3f} m"
            )
            self.status.config(text=msg)

        except Exception as e:
            messagebox.showerror("Error", str(e))
            self.status.config(text=f"Error: {e}")

    def save_csv(self):
        if self.last_data is None:
            messagebox.showwarning("No data", "Plot trajectories first.")
            return

        p = filedialog.asksaveasfilename(
            title="Save aligned trajectory CSV",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        if not p:
            return

        d = self.last_data
        arr = np.column_stack([
            d["t"],
            d["gt"],
            d["est_aligned"],
            d["est_raw"],
            d["err"],
        ])

        header = (
            "timestamp,"
            "gt_x,gt_y,gt_z,"
            "est_aligned_x,est_aligned_y,est_aligned_z,"
            "est_raw_x,est_raw_y,est_raw_z,"
            "error_m"
        )

        np.savetxt(p, arr, delimiter=",", header=header, comments="")
        self.status.config(text=f"Saved CSV: {p}")


def default_paths():
    home = Path.home()

    est_candidates = [
        home / "slam_ws/results/ntu_viral_eee03_60s/CameraTrajectory.txt",
        home / "slam_ws/mono_slam/CameraTrajectory.txt",
    ]

    gt_candidates = [
        home / "slam_ws/results/ntu_viral_eee03_60s/groundtruth_leica_raw.txt",
        home / "slam_ws/datasets/ntu_viral/eee_03/eee_03/eee_03.bag",
    ]

    est = next((p for p in est_candidates if p.exists()), est_candidates[0])
    gt = next((p for p in gt_candidates if p.exists()), gt_candidates[0])

    return est, gt


def main():
    est_default, gt_default = default_paths()

    ap = argparse.ArgumentParser()
    ap.add_argument("--est", default=str(est_default))
    ap.add_argument("--gt", default=str(gt_default))
    args = ap.parse_args()

    root = tk.Tk()
    app = TrajectoryGUI(root, args.est, args.gt)
    root.mainloop()


if __name__ == "__main__":
    main()

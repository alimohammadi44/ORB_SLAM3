#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from tqdm import tqdm


def stamp_to_ns(stamp):
    if hasattr(stamp, "sec"):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if hasattr(stamp, "secs"):
        return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
    raise RuntimeError(f"Unknown stamp format: {stamp}")


def msg_time_ns(msg, fallback_ns):
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        return stamp_to_ns(msg.header.stamp)
    return int(fallback_ns)


def image_to_numpy(msg):
    enc = msg.encoding.lower()
    h = int(msg.height)
    w = int(msg.width)
    step = int(msg.step)

    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if enc in ["mono8", "8uc1"]:
        img = data.reshape(h, step)[:, :w]
        return img

    if enc in ["bgr8", "rgb8"]:
        img = data.reshape(h, step)[:, : w * 3].reshape(h, w, 3)
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return gray

    if enc in ["mono16", "16uc1"]:
        data16 = np.frombuffer(bytes(msg.data), dtype=np.uint16)
        img16 = data16.reshape(h, step // 2)[:, :w]
        img8 = cv2.convertScaleAbs(img16, alpha=(255.0 / max(1, img16.max())))
        return img8

    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--image-topic", default="/left/image_raw")
    ap.add_argument("--imu-topic", default="/imu/imu")
    ap.add_argument("--max-images", type=int, default=600,
                    help="Use 0 for all images. 600 images is about 60 seconds at 10 Hz.")
    ap.add_argument("--duration-sec", type=float, default=0.0,
                    help="Optional duration limit from first selected image. 0 disables.")
    ap.add_argument("--imu-pre-sec", type=float, default=2.0)
    args = ap.parse_args()

    bag = Path(args.bag).expanduser()
    out = Path(args.out).expanduser()

    cam_dir = out / "mav0" / "cam0" / "data"
    imu_dir = out / "mav0" / "imu0"
    cam_dir.mkdir(parents=True, exist_ok=True)
    imu_dir.mkdir(parents=True, exist_ok=True)

    times_file = out / "times.txt"
    imu_csv = imu_dir / "data.csv"

    print(f"Input bag: {bag}")
    print(f"Output:    {out}")
    print(f"Image:     {args.image_topic}")
    print(f"IMU:       {args.imu_topic}")

    # Pass 1: determine image time window.
    image_times = []
    with AnyReader([bag]) as reader:
        img_conns = [c for c in reader.connections if c.topic == args.image_topic]
        if not img_conns:
            raise RuntimeError(f"Image topic not found: {args.image_topic}")

        for conn, timestamp, rawdata in reader.messages(connections=img_conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            t_ns = msg_time_ns(msg, timestamp)

            if not image_times:
                start_ns = t_ns

            if args.duration_sec > 0 and t_ns > start_ns + int(args.duration_sec * 1e9):
                break

            image_times.append(t_ns)

            if args.max_images > 0 and len(image_times) >= args.max_images:
                break

    if not image_times:
        raise RuntimeError("No images selected.")

    start_ns = image_times[0]
    end_ns = image_times[-1]
    imu_start_ns = start_ns - int(args.imu_pre_sec * 1e9)

    print(f"Selected images: {len(image_times)}")
    print(f"Start ns: {start_ns}")
    print(f"End ns:   {end_ns}")

    selected_set = set(image_times)

    # Pass 2: write images and IMU.
    n_img = 0
    n_imu = 0

    with times_file.open("w") as ft, imu_csv.open("w", newline="") as fi:
        writer = csv.writer(fi)
        writer.writerow([
            "#timestamp [ns]",
            "w_RS_S_x [rad s^-1]",
            "w_RS_S_y [rad s^-1]",
            "w_RS_S_z [rad s^-1]",
            "a_RS_S_x [m s^-2]",
            "a_RS_S_y [m s^-2]",
            "a_RS_S_z [m s^-2]",
        ])

        with AnyReader([bag]) as reader:
            conns = [
                c for c in reader.connections
                if c.topic in [args.image_topic, args.imu_topic]
            ]

            for conn, timestamp, rawdata in tqdm(reader.messages(connections=conns), desc="Extracting"):
                msg = reader.deserialize(rawdata, conn.msgtype)
                t_ns = msg_time_ns(msg, timestamp)

                if conn.topic == args.image_topic:
                    if t_ns not in selected_set:
                        continue

                    img = image_to_numpy(msg)
                    out_img = cam_dir / f"{t_ns}.png"
                    ok = cv2.imwrite(str(out_img), img)
                    if not ok:
                        raise RuntimeError(f"Failed to write image: {out_img}")

                    ft.write(f"{t_ns}\n")
                    n_img += 1

                elif conn.topic == args.imu_topic:
                    if t_ns < imu_start_ns or t_ns > end_ns:
                        continue

                    av = msg.angular_velocity
                    la = msg.linear_acceleration

                    writer.writerow([
                        t_ns,
                        av.x, av.y, av.z,
                        la.x, la.y, la.z,
                    ])
                    n_imu += 1

    print()
    print("Done.")
    print(f"Images written: {n_img}")
    print(f"IMU rows:       {n_imu}")
    print(f"Times file:     {times_file}")
    print(f"IMU CSV:        {imu_csv}")
    print()
    print("Output layout:")
    print(f"{out}/mav0/cam0/data/*.png")
    print(f"{out}/mav0/imu0/data.csv")
    print(f"{out}/times.txt")


if __name__ == "__main__":
    main()

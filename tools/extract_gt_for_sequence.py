#!/usr/bin/env python3
import argparse
from pathlib import Path
from rosbags.highlevel import AnyReader

def stamp_to_ns(stamp):
    if hasattr(stamp, "sec"):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if hasattr(stamp, "secs"):
        return int(stamp.secs) * 1_000_000_000 + int(stamp.nsecs)
    raise RuntimeError("Unknown stamp format")

def msg_time_sec(msg, fallback_ns):
    if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
        return stamp_to_ns(msg.header.stamp) / 1e9
    return int(fallback_ns) / 1e9

def load_times_sec(path):
    vals = []
    for line in Path(path).expanduser().read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        vals.append(float(line))
    if max(vals) > 1e12:   # nanoseconds
        vals = [v / 1e9 for v in vals]
    return min(vals), max(vals)

ap = argparse.ArgumentParser()
ap.add_argument("--bag", required=True)
ap.add_argument("--times", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--topic", default="/leica/pose/relative")
ap.add_argument("--margin", type=float, default=2.0)
args = ap.parse_args()

bag = Path(args.bag).expanduser()
out = Path(args.out).expanduser()
out.parent.mkdir(parents=True, exist_ok=True)

t0, t1 = load_times_sec(args.times)
t0 -= args.margin
t1 += args.margin

n = 0
with AnyReader([bag]) as reader, out.open("w") as f:
    conns = [c for c in reader.connections if c.topic == args.topic]
    if not conns:
        raise RuntimeError(f"Topic not found: {args.topic}")

    f.write("# timestamp tx ty tz qx qy qz qw\n")

    for conn, timestamp, rawdata in reader.messages(connections=conns):
        msg = reader.deserialize(rawdata, conn.msgtype)
        t = msg_time_sec(msg, timestamp)

        if t < t0 or t > t1:
            continue

        p = msg.pose.position
        q = msg.pose.orientation
        f.write(f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        n += 1

print("Selected GT time range:", t0, "to", t1)
print("Ground truth poses written:", n)
print("Output:", out)

#!/usr/bin/env python3
from pathlib import Path
from rosbags.highlevel import AnyReader
import sys

if len(sys.argv) != 2:
    print("Usage: inspect_rosbag.py path/to/file.bag")
    sys.exit(1)

bag_path = Path(sys.argv[1]).expanduser()

with AnyReader([bag_path]) as reader:
    print("Topics in bag:")
    for c in reader.connections:
        print(f"{c.topic:45s} {c.msgtype}")

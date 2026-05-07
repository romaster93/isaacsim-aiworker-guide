#!/usr/bin/env python3
"""rosbag2 (mcap) → trajectory .npz extractor."""

import argparse
import math
import sys
from pathlib import Path

import numpy as np


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def open_reader(bag_dir: str):
    try:
        import rosbag2_py
        from rosbag2_py import StorageOptions, ConverterOptions
    except ImportError:
        print("ERROR: rosbag2_py not found. Source ros2-bridge-env.sh first.", file=sys.stderr)
        sys.exit(1)

    bag_path = Path(bag_dir)
    if not bag_path.exists():
        print(f"ERROR: bag path does not exist: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    if not (bag_path / "metadata.yaml").exists():
        print(f"ERROR: metadata.yaml not found in {bag_dir}", file=sys.stderr)
        sys.exit(1)

    reader = rosbag2_py.SequentialReader()
    storage_opts = StorageOptions(uri=str(bag_path), storage_id="mcap")
    conv_opts = ConverterOptions("", "")
    reader.open(storage_opts, conv_opts)
    return reader


def read_bag(bag_dir: str, label: str, out_path: str) -> None:
    from rclpy.serialization import deserialize_message
    from sensor_msgs_py import point_cloud2
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Int32

    reader = open_reader(bag_dir)

    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}

    ODOM_TOPIC = "/habitat/odom"
    OCC_TOPIC = "/grid_map/occupied"
    FREE_TOPIC = "/grid_map/free"
    GOAL_TOPIC = "/move_base_simple/goal"
    STATE_TOPIC = "/ros/expl_state"

    if ODOM_TOPIC not in topic_types:
        print(f"ERROR: required topic {ODOM_TOPIC} not found in bag.", file=sys.stderr)
        sys.exit(1)

    type_map = {
        ODOM_TOPIC: Odometry,
        OCC_TOPIC: PointCloud2,
        FREE_TOPIC: PointCloud2,
        GOAL_TOPIC: PoseStamped,
        STATE_TOPIC: Int32,
    }

    traj_rows: list = []
    last_occ_msg = None
    last_free_msg = None
    t_start = float("nan")
    t_finish = float("nan")
    prev_state = None

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in type_map:
            continue

        msg_type = type_map[topic]
        msg = deserialize_message(data, msg_type)

        if topic == ODOM_TOPIC:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            traj_rows.append([ts, p.x, p.y, yaw])

        elif topic == OCC_TOPIC:
            last_occ_msg = msg

        elif topic == FREE_TOPIC:
            last_free_msg = msg

        elif topic == GOAL_TOPIC:
            if math.isnan(t_start):
                ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                t_start = ts

        elif topic == STATE_TOPIC:
            val = msg.data
            if prev_state != 5 and val == 5 and math.isnan(t_finish):
                if traj_rows:
                    t_finish = traj_rows[-1][0]
            prev_state = val

    if not traj_rows:
        print("ERROR: /habitat/odom had no messages.", file=sys.stderr)
        sys.exit(1)

    traj = np.array(traj_rows, dtype=np.float64)

    def extract_xy(pc_msg):
        if pc_msg is None:
            return None
        pts = list(point_cloud2.read_points(pc_msg, field_names=["x", "y"], skip_nans=True))
        if not pts:
            return None
        return np.array([[p[0], p[1]] for p in pts], dtype=np.float32)

    occ_xy = extract_xy(last_occ_msg)
    free_xy = extract_xy(last_free_msg)

    duration = traj[-1, 0] - traj[0, 0] if len(traj) > 1 else 0.0

    save_kwargs: dict = {
        "traj": traj,
        "label": np.array(label),
        "bag_path": np.array(str(Path(bag_dir).resolve())),
        "duration": np.array(duration),
        "t_start": np.array(t_start),
        "t_finish": np.array(t_finish),
    }
    if occ_xy is not None:
        save_kwargs["occupied_xy"] = occ_xy
    else:
        save_kwargs["occupied_xy"] = np.array(None, dtype=object)
    if free_xy is not None:
        save_kwargs["free_xy"] = free_xy
    else:
        save_kwargs["free_xy"] = np.array(None, dtype=object)

    np.savez(out_path, **save_kwargs)
    print(f"Saved: {out_path}")
    print(f"  traj shape:    {traj.shape}")
    print(f"  occupied_xy:   {occ_xy.shape if occ_xy is not None else None}")
    print(f"  free_xy:       {free_xy.shape if free_xy is not None else None}")
    print(f"  t_start:       {t_start}")
    print(f"  t_finish:      {t_finish}")
    print(f"  duration:      {duration:.2f}s")


def main() -> None:
    parser = argparse.ArgumentParser(description="rosbag2 (mcap) → trajectory .npz")
    parser.add_argument("bag_dir", help="Path to rosbag2 directory (contains metadata.yaml)")
    parser.add_argument("-o", "--output", required=True, help="Output .npz path")
    parser.add_argument("--label", default="run", help="Label string stored in .npz")
    args = parser.parse_args()

    read_bag(args.bag_dir, args.label, args.output)


if __name__ == "__main__":
    main()

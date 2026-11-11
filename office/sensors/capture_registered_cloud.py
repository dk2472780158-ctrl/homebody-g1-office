"""Bounded, read-only capture of FAST-LIO registered clouds and odometry."""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


def xyz_array(msg):
    fields = {field.name: field for field in msg.fields}
    if any(name not in fields or fields[name].datatype != PointField.FLOAT32
           for name in ("x", "y", "z")):
        raise ValueError("Cloud XYZ fields must be float32")
    dtype = ">f4" if msg.is_bigendian else "<f4"
    coords = [np.ndarray((msg.height, msg.width), dtype=dtype,
                         buffer=msg.data, offset=fields[name].offset,
                         strides=(msg.row_step, msg.point_step)).reshape(-1)
              for name in ("x", "y", "z")]
    result = np.column_stack(coords).astype(np.float32)
    result = result[np.isfinite(result).all(axis=1)]
    if len(result) > 40000:
        result = result[np.linspace(0, len(result)-1, 40000, dtype=int)]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=32)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node("homebody_readonly_cloud_capture")
    started = time.monotonic()
    frames, frame_meta, odometry = [], [], []
    last_cloud, last_odom = -1e12, -1e12
    stats = {name: {"messages": 0, "first_stamp_s": None, "last_stamp_s": None,
                    "nonincreasing_stamps": 0, "max_stamp_gap_s": 0.}
             for name in ("cloud_registered", "odometry")}

    def observe(name, msg):
        stat = stats[name]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stat["last_stamp_s"] is not None:
            gap = stamp-stat["last_stamp_s"]
            stat["max_stamp_gap_s"] = max(stat["max_stamp_gap_s"], gap)
            stat["nonincreasing_stamps"] += int(gap <= 0)
        else:
            stat["first_stamp_s"] = stamp
        stat["last_stamp_s"] = stamp
        stat["messages"] += 1

    def cloud_callback(msg):
        nonlocal last_cloud
        observe("cloud_registered", msg)
        now = time.monotonic()
        if now-last_cloud < 0.9 or len(frames) >= 28:
            return
        points = xyz_array(msg)
        if len(points) < 100:
            return
        last_cloud = now
        frames.append(points)
        frame_meta.append({"stamp_sec": msg.header.stamp.sec,
                           "stamp_nanosec": msg.header.stamp.nanosec,
                           "frame_id": msg.header.frame_id,
                           "points": len(points),
                           "elapsed_s": now-started})

    def odom_callback(msg):
        nonlocal last_odom
        observe("odometry", msg)
        now = time.monotonic()
        last_odom = now
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        odometry.append({"stamp_sec": msg.header.stamp.sec,
                         "stamp_nanosec": msg.header.stamp.nanosec,
                         "frame_id": msg.header.frame_id,
                         "child_frame_id": msg.child_frame_id,
                         "xyz": [p.x, p.y, p.z],
                         "xyzw": [q.x, q.y, q.z, q.w],
                         "elapsed_s": now-started})

    node.create_subscription(PointCloud2, "/cloud_registered", cloud_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/Odometry", odom_callback, qos_profile_sensor_data)
    deadline = started + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    arrays = {f"frame_{i:02d}": arr for i, arr in enumerate(frames)}
    if frames:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(args.output, **arrays)
    report = {"scope": "read-only FAST-LIO cloud/odometry subscription",
              "duration_s": args.duration, "clouds": frame_meta,
              "odometry": odometry, "stats": stats,
              "archive": str(args.output) if frames else None}
    args.output.with_suffix(".json").write_text(json.dumps(report, indent=2))
    print(json.dumps({"cloud_frames": len(frames), "odometry_samples": len(odometry),
                      "cloud_frame_ids": sorted({m["frame_id"] for m in frame_meta}),
                      "odom_frame_ids": sorted({m["frame_id"] for m in odometry}),
                      "archive": report["archive"]}), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

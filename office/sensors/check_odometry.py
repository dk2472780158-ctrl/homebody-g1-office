"""Bounded, read-only ROS 2 observation of FAST-LIO output."""
import argparse
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15)
    args = parser.parse_args()
    rclpy.init()
    node = Node("g1_readonly_odometry_check")
    report = {"count": 0, "first": None, "last": None, "scope": "read-only /Odometry subscription"}

    def received(msg):
        pose = msg.pose.pose
        record = {"receive_mono_s": time.monotonic(), "stamp_sec": msg.header.stamp.sec,
                  "stamp_nanosec": msg.header.stamp.nanosec, "frame": msg.header.frame_id,
                  "child_frame": msg.child_frame_id,
                  "xyz_m": [pose.position.x, pose.position.y, pose.position.z],
                  "quaternion": [pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w]}
        if report["first"] is None:
            report["first"] = record
        report["last"] = record
        report["count"] += 1

    node.create_subscription(Odometry, "/Odometry", received, qos_profile_sensor_data)
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.25)
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

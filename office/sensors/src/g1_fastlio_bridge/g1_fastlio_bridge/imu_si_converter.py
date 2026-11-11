#!/usr/bin/env python3
import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class ImuSiConverter(Node):
    """Republish G1 Mid360 IMU with a usable frame id and SI acceleration."""

    def __init__(self):
        super().__init__("g1_mid360_imu_si_converter")

        self.input_topic = self.declare_parameter(
            "input_topic", "/utlidar/imu_livox_mid360"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/g1_fastlio/imu_livox_mid360_si"
        ).value
        self.target_frame_id = self.declare_parameter(
            "target_frame_id", "body"
        ).value
        self.rotate_roll_pi = _as_bool(
            self.declare_parameter("rotate_roll_pi", True).value
        )
        self.accel_scale = float(self.declare_parameter("accel_scale", 9.80665).value)
        self.auto_scale_accel = _as_bool(
            self.declare_parameter("auto_scale_accel", False).value
        )
        self.raw_g_norm_max = float(
            self.declare_parameter("raw_g_norm_max", 3.0).value
        )
        self.raw_g_norm_min = float(
            self.declare_parameter("raw_g_norm_min", 0.2).value
        )

        input_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        output_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(Imu, self.output_topic, output_qos)
        self.subscription = self.create_subscription(
            Imu, self.input_topic, self._imu_callback, input_qos
        )
        self._last_reported_scale = None

        self.get_logger().info(
            f"Converting IMU {self.input_topic} -> {self.output_topic}, "
            f"frame_id={self.target_frame_id}, "
            f"auto_scale_accel={self.auto_scale_accel}, "
            f"rotate_roll_pi={self.rotate_roll_pi}"
        )

    def _imu_callback(self, msg: Imu) -> None:
        out = copy.deepcopy(msg)
        if self.target_frame_id and not out.header.frame_id:
            out.header.frame_id = self.target_frame_id

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        norm = math.sqrt(ax * ax + ay * ay + az * az)

        scale = self.accel_scale
        if self.auto_scale_accel:
            scale = (
                self.accel_scale
                if self.raw_g_norm_min < norm < self.raw_g_norm_max
                else 1.0
            )

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        if self.rotate_roll_pi:
            ay = -ay
            az = -az
            gy = -gy
            gz = -gz

        out.linear_acceleration.x = ax * scale
        out.linear_acceleration.y = ay * scale
        out.linear_acceleration.z = az * scale
        out.angular_velocity.x = gx
        out.angular_velocity.y = gy
        out.angular_velocity.z = gz

        if self._last_reported_scale != scale:
            self._last_reported_scale = scale
            self.get_logger().info(
                f"IMU acceleration norm {norm:.3f}, "
                f"publishing with scale {scale:.5f}"
            )

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuSiConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

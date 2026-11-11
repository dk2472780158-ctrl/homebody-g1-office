#!/usr/bin/env python3
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class CloudRollPiConverter(Node):
    """Republish PointCloud2 after a 180 degree roll around x."""

    def __init__(self):
        super().__init__("g1_mid360_cloud_roll_pi_converter")

        self.input_topic = self.declare_parameter(
            "input_topic", "/utlidar/cloud_livox_mid360"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/g1_fastlio/cloud_livox_mid360_upright"
        ).value
        self.target_frame_id = self.declare_parameter("target_frame_id", "body").value
        self.rotate_roll_pi = _as_bool(
            self.declare_parameter("rotate_roll_pi", True).value
        )
        self.livox_timestamp_unit = self.declare_parameter(
            "livox_timestamp_unit", "auto"
        ).value

        input_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        output_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, output_qos
        )
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, input_qos
        )
        self._field_offsets = None
        self._livox_layout = None
        self._reported_timestamp_scale = None

        self.get_logger().info(
            f"Converting cloud {self.input_topic} -> {self.output_topic}, "
            f"frame_id={self.target_frame_id}, rotate_roll_pi={self.rotate_roll_pi}"
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        field_map = {field.name: field for field in msg.fields}
        if "line" in field_map and "timestamp" in field_map:
            self._publish_livox_cloud(msg, field_map)
            return

        out = PointCloud2()
        out.header = msg.header
        if self.target_frame_id:
            out.header.frame_id = self.target_frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_dense = msg.is_dense
        out.data = bytes(msg.data)

        if self.rotate_roll_pi:
            data = bytearray(out.data)
            if self._field_offsets is None:
                offsets = {field.name: field.offset for field in msg.fields}
                self._field_offsets = (
                    offsets.get("x"),
                    offsets.get("y"),
                    offsets.get("z"),
                )
                self.get_logger().info(
                    f"Point field offsets x/y/z={self._field_offsets}"
                )

            x_off, y_off, z_off = self._field_offsets
            if x_off is None or y_off is None or z_off is None:
                self.get_logger().error("PointCloud2 lacks x/y/z fields")
                return

            total = msg.width * msg.height
            step = msg.point_step
            endian = ">" if msg.is_bigendian else "<"
            fmt = endian + "f"
            for i in range(total):
                base = i * step
                y = struct.unpack_from(fmt, data, base + y_off)[0]
                z = struct.unpack_from(fmt, data, base + z_off)[0]
                struct.pack_into(fmt, data, base + y_off, -y)
                struct.pack_into(fmt, data, base + z_off, -z)
            out.data = bytes(data)

        self.publisher.publish(out)

    def _publish_livox_cloud(self, msg: PointCloud2, field_map) -> None:
        """Convert Livox PointCloud2 fields to FAST-LIO's ring/time layout."""
        required_types = {
            "x": PointField.FLOAT32,
            "y": PointField.FLOAT32,
            "z": PointField.FLOAT32,
            "intensity": PointField.FLOAT32,
            "line": PointField.UINT8,
            "timestamp": PointField.FLOAT64,
        }
        if any(
            name not in field_map or field_map[name].datatype != datatype
            for name, datatype in required_types.items()
        ):
            self.get_logger().error(
                "Unsupported Livox PointCloud2 layout; expected x/y/z/intensity "
                "(float32), line (uint8), timestamp (float64)"
            )
            return

        if self._livox_layout is None:
            self._livox_layout = tuple(
                field_map[name].offset for name in required_types
            )
            self.get_logger().info(
                "Converting Livox fields line:uint8 -> ring:uint16 and "
                "timestamp:float64 -> time:float32"
            )

        x_off, y_off, z_off, intensity_off, line_off, timestamp_off = (
            self._livox_layout
        )
        first_timestamp = struct.unpack_from(
            ">d" if msg.is_bigendian else "<d", msg.data, timestamp_off
        )[0]
        timestamp_scale = self._timestamp_scale(first_timestamp, msg)

        out = PointCloud2()
        out.header = msg.header
        if self.target_frame_id:
            out.header.frame_id = self.target_frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=18, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 22
        out.row_step = out.point_step * out.width
        out.is_dense = msg.is_dense
        data = bytearray(out.row_step * out.height)
        input_endian = ">" if msg.is_bigendian else "<"

        for row in range(msg.height):
            for column in range(msg.width):
                input_base = row * msg.row_step + column * msg.point_step
                output_base = row * out.row_step + column * out.point_step
                x = struct.unpack_from(input_endian + "f", msg.data, input_base + x_off)[0]
                y = struct.unpack_from(input_endian + "f", msg.data, input_base + y_off)[0]
                z = struct.unpack_from(input_endian + "f", msg.data, input_base + z_off)[0]
                intensity = struct.unpack_from(
                    input_endian + "f", msg.data, input_base + intensity_off
                )[0]
                line = struct.unpack_from(input_endian + "B", msg.data, input_base + line_off)[0]
                timestamp = struct.unpack_from(
                    input_endian + "d", msg.data, input_base + timestamp_off
                )[0]
                if self.rotate_roll_pi:
                    y = -y
                    z = -z
                struct.pack_into(
                    "<ffffHf",
                    data,
                    output_base,
                    x,
                    y,
                    z,
                    intensity,
                    line,
                    (timestamp - first_timestamp) * timestamp_scale,
                )

        out.data = bytes(data)
        self.publisher.publish(out)

    def _timestamp_scale(self, timestamp: float, msg: PointCloud2) -> float:
        units = {
            "seconds": 1.0,
            "milliseconds": 1e-3,
            "microseconds": 1e-6,
            "nanoseconds": 1e-9,
        }
        if self.livox_timestamp_unit in units:
            return units[self.livox_timestamp_unit]

        header_seconds = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        scale = 1e-9 if abs(timestamp) > 1e5 else 1.0
        if header_seconds > 0:
            if abs(timestamp - header_seconds) < 10.0:
                scale = 1.0
            elif abs(timestamp - header_seconds * 1e9) < 1e10:
                scale = 1e-9
        if self._reported_timestamp_scale != scale:
            self._reported_timestamp_scale = scale
            self.get_logger().info(
                f"Livox timestamp scale selected as {scale} seconds per unit"
            )
        return scale


def main(args=None):
    rclpy.init(args=args)
    node = CloudRollPiConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

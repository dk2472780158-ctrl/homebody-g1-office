from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_imu_topic = LaunchConfiguration("input_imu_topic")
    output_imu_topic = LaunchConfiguration("output_imu_topic")
    input_cloud_topic = LaunchConfiguration("input_cloud_topic")
    output_cloud_topic = LaunchConfiguration("output_cloud_topic")
    lidar_frame = LaunchConfiguration("lidar_frame")
    base_frame = LaunchConfiguration("base_frame")
    raw_lidar_frame = LaunchConfiguration("raw_lidar_frame")
    publish_static_tf = LaunchConfiguration("publish_static_tf")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_imu_topic", default_value="/utlidar/imu_livox_mid360"
            ),
            DeclareLaunchArgument(
                "output_imu_topic",
                default_value="/g1_fastlio/imu_livox_mid360_upright",
            ),
            DeclareLaunchArgument(
                "input_cloud_topic", default_value="/utlidar/cloud_livox_mid360"
            ),
            DeclareLaunchArgument(
                "output_cloud_topic",
                default_value="/g1_fastlio/cloud_livox_mid360_upright",
            ),
            DeclareLaunchArgument("lidar_frame", default_value="body"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("raw_lidar_frame", default_value="livox_frame"),
            DeclareLaunchArgument("auto_scale_accel", default_value="false"),
            DeclareLaunchArgument("accel_scale", default_value="9.80665"),
            DeclareLaunchArgument("rotate_roll_pi", default_value="true"),
            DeclareLaunchArgument("publish_static_tf", default_value="true"),
            # Translation is robot/mount specific. Defaults keep only orientation.
            DeclareLaunchArgument("lidar_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.0"),
            # G1 Mid360 is mounted upside down with x kept forward: base_link -> livox_frame
            # is a 180 degree roll. Override yaw/pitch/roll if your bracket differs.
            DeclareLaunchArgument("lidar_roll", default_value="3.141592653589793"),
            DeclareLaunchArgument("lidar_pitch", default_value="0.0"),
            DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
            Node(
                package="g1_fastlio_bridge",
                executable="cloud_roll_pi_converter",
                name="g1_mid360_cloud_roll_pi_converter",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_cloud_topic,
                        "output_topic": output_cloud_topic,
                        "target_frame_id": lidar_frame,
                        "rotate_roll_pi": LaunchConfiguration("rotate_roll_pi"),
                    }
                ],
            ),
            Node(
                package="g1_fastlio_bridge",
                executable="imu_si_converter",
                name="g1_mid360_imu_si_converter",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_imu_topic,
                        "output_topic": output_imu_topic,
                        "target_frame_id": lidar_frame,
                        "auto_scale_accel": LaunchConfiguration("auto_scale_accel"),
                        "accel_scale": LaunchConfiguration("accel_scale"),
                        "rotate_roll_pi": LaunchConfiguration("rotate_roll_pi"),
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="g1_body_base_static_tf",
                condition=IfCondition(publish_static_tf),
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    lidar_frame,
                    "--child-frame-id",
                    base_frame,
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="g1_raw_livox_static_tf",
                condition=IfCondition(publish_static_tf),
                arguments=[
                    "--x",
                    LaunchConfiguration("lidar_x"),
                    "--y",
                    LaunchConfiguration("lidar_y"),
                    "--z",
                    LaunchConfiguration("lidar_z"),
                    "--roll",
                    LaunchConfiguration("lidar_roll"),
                    "--pitch",
                    LaunchConfiguration("lidar_pitch"),
                    "--yaw",
                    LaunchConfiguration("lidar_yaw"),
                    "--frame-id",
                    base_frame,
                    "--child-frame-id",
                    raw_lidar_frame,
                ],
            ),
        ]
    )

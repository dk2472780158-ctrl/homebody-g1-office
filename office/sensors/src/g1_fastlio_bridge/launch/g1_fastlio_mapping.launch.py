import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path_default = os.path.join(
        get_package_share_directory("g1_fastlio_bridge"), "config"
    )

    preprocess_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("g1_fastlio_bridge"),
            "launch",
            "g1_fastlio_preprocess.launch.py",
        )
    )
    fast_lio_launch = PythonLaunchDescriptionSource(
        [
            FindPackageShare(LaunchConfiguration("fast_lio_package")),
            "/launch/",
            LaunchConfiguration("fast_lio_launch_file"),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("fast_lio_package", default_value="fast_lio"),
            DeclareLaunchArgument("fast_lio_launch_file", default_value="mapping.launch.py"),
            DeclareLaunchArgument("fast_lio_config_path", default_value=config_path_default),
            DeclareLaunchArgument("fast_lio_config_file", default_value="g1_mid360_fastlio.yaml"),
            DeclareLaunchArgument(
                "map_file_path",
                default_value="src/map/g1_fastlio.pcd",
                description="Relative or absolute PCD output path",
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            IncludeLaunchDescription(preprocess_launch),
            IncludeLaunchDescription(
                fast_lio_launch,
                launch_arguments={
                    "config_path": LaunchConfiguration("fast_lio_config_path"),
                    "config_file": LaunchConfiguration("fast_lio_config_file"),
                    "map_file_path": LaunchConfiguration("map_file_path"),
                    "rviz": LaunchConfiguration("rviz"),
                }.items(),
            ),
        ]
    )

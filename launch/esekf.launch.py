from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")

    default_config = os.path.join(
        get_package_share_directory("mavtech_filter"),
        "config",
        "esekf.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to mavtech_filter YAML config",
        ),

        Node(
            package="mavtech_filter",
            executable="esekf_node",
            name="esekf_node",
            output="screen",
            parameters=[config_file],
        ),
    ])

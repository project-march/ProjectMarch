"""Author: Tuhin, MVII."""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """Launch file to launch wireless input device. The IPD can be started by setting the wireless_ipd ROS2 argument to True."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="node_prefix",
                default_value=[EnvironmentVariable("USER"), "_"],
                description="Prefix for node names",
            ),
            Node(
                package="march_wireless_ipd",
                executable="march_wireless_ipd_node",
                output="screen",
                name="wireless_ipd",
                namespace="march",
            ),
        ]
    )

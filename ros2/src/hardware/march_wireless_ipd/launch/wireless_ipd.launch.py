"""Author: Tuhin, MVII."""

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> launch.LaunchDescription:
    """Launch file to launch wireless input device. The IPD can be started by setting the wireless_ipd ROS2 argument to True."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="ip_address",
                default_value="192.168.0.100",
                description="IP address on which the IPD can connect."
            ),
            Node(
                package="march_wireless_ipd",
                executable="march_wireless_ipd_node",
                output="screen",
                name="wireless_ipd",
                namespace="march",
                respawn=True,
                parameters=[
                    {"ip_address": LaunchConfiguration("ip_address")},
                ],
            ),
        ]
    )

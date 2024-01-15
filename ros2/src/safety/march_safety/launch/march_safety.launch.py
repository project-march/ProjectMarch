"""Launch description for safety Node."""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    
    return LaunchDescription(
        [
            Node(
                package="march_safety",
                executable="march_safety_node",
                name="safety_node",
                namespace="march",
                output="screen",
                on_exit=Shutdown(),
            ),
        ]
    )

"""Author: MARCH."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""

    return LaunchDescription([
        Node(
            package='fuzzy_generator',
            namespace='',
            executable='fuzzy_node',
            name='fuzzy_generator'
        )
    ])

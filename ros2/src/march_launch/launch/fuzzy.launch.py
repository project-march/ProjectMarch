"""Author: MARCH."""
from launch import LaunchDescription

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

"""Author: MVIII."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the gait loader.

    This node is started when the gait loader is run
    """
    return LaunchDescription([
        Node(
            package='gait_loader',
            executable='gait_loader_node',
            name='gait_loader',
        ),
    ])

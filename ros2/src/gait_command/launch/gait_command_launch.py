"""Author: MVIII."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the state estimator.

    This node is started when the state estimator is run
    """
    return LaunchDescription([
        Node(
            package='gait_command',
            namespace='',
            executable='gait_command_node',
            name='gait_command',
        ),
    ])

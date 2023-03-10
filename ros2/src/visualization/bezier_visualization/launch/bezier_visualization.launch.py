"""Author: MVIII."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch description of the mujoco simulation nodes.

    These nodes are started when the mujoco simulation has be run.
    """
    return LaunchDescription([
        Node(
            package='bezier_visualization',
            executable='bezier_visualization_node',
            name='bezier_visualization',
        ),
    ])

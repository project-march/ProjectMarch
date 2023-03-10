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
            package='swing_leg_trajectory_generator',
            namespace='',
            executable='swing_leg_trajectory_generator_node',
            name='swing_leg_trajectory_generator',
        ),
    ])

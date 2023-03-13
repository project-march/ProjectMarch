"""Author: MVIII."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the swing leg trajectory generator node.

    This node is started when the swing leg trajectory generator  has be run.
    """
    return LaunchDescription([
        Node(
            package='swing_leg_trajectory_generator',
            namespace='',
            executable='swing_leg_trajectory_generator_node',
            name='swing_leg_trajectory_generator',
        ),
    ])

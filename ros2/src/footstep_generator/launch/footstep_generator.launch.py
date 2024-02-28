"""Author: MVIII."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the ik solver.

    This node is started when the ik solver is ran
    """
    # parameters
    n_footsteps = LaunchConfiguration("n_footsteps", default='20')
    step_length = LaunchConfiguration("step_length", default='0.2')

    return LaunchDescription([
        Node(
            package='footstep_generator',
            namespace='',
            executable='footstep_generator_node',
            name='footstep_generator',
            parameters=[
                {"n_footsteps", n_footsteps},
                {"step_length", step_length},
            ]
        ),
    ])

"""Author: MARCH."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""

    return LaunchDescription([
        Node(
            package='footstep_generator',
            namespace='',
            executable='footstep_generator_node',
            name='footstep_generator'
        ),
        Node(
            package='swing_leg_trajectory_generator',
            namespace='',
            executable='swing_leg_trajectory_generator_node',
            name='swing_leg_generator'
        ),
        Node(
            package='zmp_mpc_solver',
            namespace='',
            executable='zmp_mpc_solver',
            name='zmp_mpc_solver'
        ),
        Node(
            package='state_estimator',
            namespace='',
            executable='state_estimator_node',
            name='state_estimator'
        ),
        
    ])

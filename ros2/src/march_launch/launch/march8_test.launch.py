"""Author: MARCH."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    # region Launch Mujoco
    mujoco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("mujoco_sim"), "mujoco_sim.launch.py"])]
        ),
    )

    return LaunchDescription([
        Node(
            package='state_estimator',
            namespace='',
            executable='state_estimator_node',
            name='state_estimator'
        ),
        Node(
            package='acados_solver',
            namespace='',
            executable='solver_node',
            name='acados_solver'
        ),
        Node(
            package='gait_command',
            executable='gait_command_node',
            name='gait_command',
        ),
        Node(
            package='state_machine',
            executable='state_machine_node',
            name='state_machine'
        ),
        Node(
            package='gait_loader',
            executable='gait_loader_node',
            name='gait_loader'
        ),
        mujoco_node,
    ])

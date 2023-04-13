"""Author: MARCH."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    ik_solver_launch_dir = os.path.join(
        get_package_share_directory('ik_solver'),
        'launch'
    )

    state_estimator_launch_dir = os.path.join(
        get_package_share_directory('state_estimator'),
        'launch'
    )

    urdf_location = os.path.join(
        get_package_share_directory('march_description'),
        'urdf',
        'march7_FROST.urdf'
    )

    # declare parameters
    # trajectory_dt=0.008
    # trajectory_timestep = DeclareLaunchArgument(
        # name="timestep",
        # default_value=trajectory_dt,
        # description="timestep for the zmp and ik solver trajectory",
    # )

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
            name='zmp_mpc_solver',
            # parameters=[
            #     # {"timestep": trajectory_dt}
            # ],
        ),
        Node(
            package='ik_solver_buffer',
            namespace='',
            executable='ik_solver_buffer_node',
            name='ik_solver_buffer'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ik_solver_launch_dir, '/ik_solver_launch.py']),
            launch_arguments={'robot_description': urdf_location}.items(),
                # ("timestep", trajectory_timestep),
            
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_estimator_launch_dir, '/state_estimator_launch.py']),
        ),
    ])

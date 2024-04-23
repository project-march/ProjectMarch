"""

Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # config = os.path.join(
    #     get_package_share_directory('march_state_estimator'),
    #     'config',
    #     'state_estimator_config.yaml'
    # )

    # robot_description = os.path.join(
    #     get_package_share_directory('march_state_estimator'),
    #     'config',
    #     'robot_definition-hennie_with_koen.yaml'
    # )
    timestep = 0.05 # in seconds
    robot_description = 'robot_definition-hennie_with_koen.yaml'
    urdf_file = os.path.join(
        get_package_share_directory('march_description'),
        'urdf',
        'march8',
        'hennie_with_koen.urdf'
    )

    return LaunchDescription([
        Node(
            package='march_state_estimator',
            executable='state_estimator_node',
            name='state_estimator',
            output='screen',
            parameters=[
                {"robot_definition": robot_description},
                {"urdf_file_path": urdf_file},
                {"timestep_in_ms": int(timestep * 1000)},
            ]
        ),
        Node(
            package='march_state_estimator',
            executable='torque_converter_node',
            name='torque_converter',
            output='screen',
            parameters=[
                {"robot_definition": robot_description},
                {"urdf_file_path": urdf_file},
            ]
        ),
        Node(
            package='march_state_estimator',
            executable='robot_description_node',
            name='robot_description',
            output='screen',
            parameters=[
                {"robot_definition": robot_description},
            ]
        ),
    ])
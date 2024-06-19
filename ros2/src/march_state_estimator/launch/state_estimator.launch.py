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
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    simulation = LaunchConfiguration("simulation", default="false")
    clock_period = LaunchConfiguration("clock_period", default="0.05")
    
    robot_description = 'robot_definition-izzy.yaml'
    urdf_file = os.path.join(
        get_package_share_directory('march_description'),
        'urdf',
        'march9',
        'march9.urdf'
    )
    force_stance_threshold = 65.0

    return LaunchDescription([
        Node(
            package='march_state_estimator',
            executable='state_estimator_node',
            name='state_estimator',
            output='screen',
            parameters=[
                {"robot_definition": robot_description},
                {"urdf_file_path": urdf_file},
                {"clock_period": clock_period},
                {"left_stance_threshold": force_stance_threshold},
                {"right_stance_threshold": force_stance_threshold},
                {"simulation": simulation},
            ],
        ),
        Node(
            package='march_state_estimator',
            executable='torque_converter_node',
            name='torque_converter',
            output='screen',
            parameters=[
                {"robot_definition": robot_description},
                {"urdf_file_path": urdf_file},
            ],
        ),
        Node(
            package='march_state_estimator',
            executable='robot_description_node',
            name='robot_description',
            output='screen',
            parameters=[
                {"robot_definition": robot_description},
            ],
        ),
        Node(
            package='march_state_estimator',
            executable='filters_node',
            name='filters',
            output='screen',
            parameters=[
                {"urdf_file_path": urdf_file},
            ],
        ),
    ])
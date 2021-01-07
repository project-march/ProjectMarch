#!/usr/bin/env python3
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_description = LaunchConfiguration('robot_description')
    xacro_path = LaunchConfiguration('xacro_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            name='robot_description',
            default_value='march4',
            description="Which <robot_description>.xacro file to use. "
                        "This file must be available in the "
                        "march_desrciption/urdf/ folder"
        ),
        DeclareLaunchArgument(
            name='xacro_path',
            default_value=[PathJoinSubstitution([
                get_package_share_directory('march_description'),
            'urdf', robot_description]), '.xacro'],
            description="Path to the xacro file to read. "
                        "If no path is supplied the robot_description argument"
                        "is used to determine the path."
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='march',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]),
    ])

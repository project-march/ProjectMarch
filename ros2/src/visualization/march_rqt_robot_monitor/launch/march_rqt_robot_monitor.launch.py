import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """ Basic launch file to launch the gait selection node """
    parameter_file = os.path.join(
        get_package_share_directory('march_rqt_robot_monitor'),
        'config', 'analyzers.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rqt',
            default_value='true',
            description='Set to launch the rqt robot monitor.'),
        Node(
            package='march_rqt_robot_monitor',
            executable='march_rqt_robot_monitor',
            output='screen',
            name='march_rqt_robot_monitor'),
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            output='screen',
            name='diag_agg',
            parameters=[parameter_file]),
        Node(
            package='rqt_robot_monitor',
            executable='rqt_robot_monitor',
            output='screen',
            name='rqt_robot_monitor',
            condition=IfCondition(LaunchConfiguration('rqt')))
    ])

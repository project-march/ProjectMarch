import os
import sys

from launch.actions import DeclareLaunchArgument, Shutdown
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Whether to use simulation time'),
        Node(
            package='march_safety',
            executable='march_safety_node',
            name='safety_node',
            namespace='march',
            output='screen',
            parameters=[
                PathJoinSubstitution([get_package_share_directory('march_safety'),
                                      'config', 'safety_settings.yaml']),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            # If this node exits, the entire system is shutdown
            # This is the ROS2 equivalent of required:=true
            # See: https://ubuntu.com/blog/ros2-launch-required-nodes
            on_exit=Shutdown()
        )
    ])
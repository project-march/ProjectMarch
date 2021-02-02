import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config_path",
                default_value=os.path.join(
                    get_package_share_directory("march_safety"),
                    "config",
                    "safety_settings.yaml",
                ),
                description="Path to the configuration for the safety settings",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="true",
                description="Whether to use simulation time",
            ),
            Node(
                package="march_safety",
                executable="march_safety_node",
                name="safety_node",
                namespace="march",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_path"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                # If this node exits, the entire system is shutdown
                # This is the ROS2 equivalent of required:=true
                # See: https://ubuntu.com/blog/ros2-launch-required-nodes
                on_exit=Shutdown(),
            ),
        ]
    )

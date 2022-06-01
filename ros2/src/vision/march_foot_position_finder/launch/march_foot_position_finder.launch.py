"""Author: Tuhin das, MVII."""
from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Todo: Add docstring."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "realsense_simulation",
                default_value="False",
                description="Whether to use the gazebo realsense plugin",
            ),
            Node(
                package="march_foot_position_finder",
                executable="march_foot_position_finder_node",
                name="march_foot_position_finder",
                namespace="march",
                output="screen",
                parameters=[
                    os.path.join(get_package_share_directory("march_foot_position_finder"), "config", "params.yaml"),
                    {"realsense_simulation": LaunchConfiguration("realsense_simulation")},
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

"""Author: Tuhin das, MVII."""
from launch import LaunchDescription
import os
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """Todo: Add docstring."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="config_path",
                default_value=os.path.join(
                    get_package_share_directory("march_foot_position_finder"),
                    "config",
                    "params.yaml",
                ),
                description="Path to the configuration for the parameters",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            Node(
                package="march_foot_position_finder",
                executable="march_foot_position_finder_node",
                name="march_foot_position_finder",
                namespace="march",
                output="screen",
                respawn=True,
                parameters=[
                    LaunchConfiguration("config_path"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

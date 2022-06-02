"""Author: Tuhin das, MVII."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Todo: Add docstring."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            Node(
                package="march_aligned_frame_publisher",
                executable="march_aligned_frame_publisher_node",
                name="march_aligned_frame_publisher",
                namespace="march",
                output="screen",
                respawn=True,
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

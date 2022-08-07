"""Author: Tuhin das, MVII."""
from launch import LaunchDescription
from launch_ros.actions import Node
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
            DeclareLaunchArgument(
                "min_check_angle",
                default_value="-5.0",
            ),
            DeclareLaunchArgument(
                "max_check_angle",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "angle_offset",
                default_value="0.1",
            ),
            DeclareLaunchArgument(
                "binary_search",
                default_value="true",
                choices=["true", "false"],
            ),
            Node(
                package="march_frame_publisher",
                executable="march_frame_publisher_node",
                name="march_frame_publisher",
                namespace="march",
                output="screen",
                respawn=True,
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"min_check_angle": LaunchConfiguration("min_check_angle")},
                    {"max_check_angle": LaunchConfiguration("max_check_angle")},
                    {"angle_offset": LaunchConfiguration("angle_offset")},
                    {"binary_search": LaunchConfiguration("binary_search")},
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

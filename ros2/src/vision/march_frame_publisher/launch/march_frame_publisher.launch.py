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
                name="rotation_camera_left",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                name="rotation_camera_right",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                name="min_check_angle",
                default_value="-10.0",
            ),
            DeclareLaunchArgument(
                name="max_check_angle",
                default_value="10.0",
            ),
            DeclareLaunchArgument(
                "angle_offset",
                default_value="0.1",
            ),
            DeclareLaunchArgument(
                name="avg_sample_size",
                default_value="10",
            ),
            DeclareLaunchArgument(
                name="num_skip_points",
                default_value="5",
            ),
            DeclareLaunchArgument(
                name="binary_steps",
                default_value="10",
            ),
            DeclareLaunchArgument(
                name="binary_search",
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
                    {"rotation_camera_left": LaunchConfiguration("rotation_camera_left")},
                    {"rotation_camera_right": LaunchConfiguration("rotation_camera_right")},
                    {"min_check_angle": LaunchConfiguration("min_check_angle")},
                    {"max_check_angle": LaunchConfiguration("max_check_angle")},
                    {"angle_offset": LaunchConfiguration("angle_offset")},
                    {"avg_sample_size": LaunchConfiguration("avg_sample_size")},
                    {"num_skip_points": LaunchConfiguration("num_skip_points")},
                    {"binary_steps": LaunchConfiguration("binary_steps")},
                    {"binary_search": LaunchConfiguration("binary_search")},
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

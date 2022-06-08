"""Author: Tuhin das, MVII."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Todo: Add docstring."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            DeclareLaunchArgument(
                name="foot_gap",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                name="step_distance",
                default_value="0.30",
            ),
            DeclareLaunchArgument(
                name="outlier_distance",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                name="sample_size",
                default_value="3",
            ),
            DeclareLaunchArgument(
                name="height_zero_threshold",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                name="foot_width",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                name="foot_length",
                default_value="0.20",
            ),
            DeclareLaunchArgument(
                name="actual_foot_length",
                default_value="0.33",
            ),
            DeclareLaunchArgument(
                name="derivative_threshold",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                name="available_points_ratio",
                default_value="0.90",
            ),
            DeclareLaunchArgument(
                name="max_z_distance",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                name="num_track_points",
                default_value="30",
            ),
            DeclareLaunchArgument(
                name="displacements_outside",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                name="displacements_inside",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                name="displacements_near",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                name="displacements_far",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "realsense_simulation",
                default_value="false",
                description="Whether to run the simulated realsense plugin.",
            ),
            Node(
                package="march_foot_position_finder",
                executable="march_foot_position_finder_node",
                name="march_foot_position_finder",
                namespace="march",
                output="screen",
                respawn=True,
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"foot_gap": LaunchConfiguration("foot_gap")},
                    {"step_distance": LaunchConfiguration("step_distance")},
                    {"outlier_distance": LaunchConfiguration("outlier_distance")},
                    {"sample_size": LaunchConfiguration("sample_size")},
                    {"height_zero_threshold": LaunchConfiguration("height_zero_threshold")},
                    {"foot_width": LaunchConfiguration("foot_width")},
                    {"foot_length": LaunchConfiguration("foot_length")},
                    {"actual_foot_length": LaunchConfiguration("actual_foot_length")},
                    {"derivative_threshold": LaunchConfiguration("derivative_threshold")},
                    {"available_points_ratio": LaunchConfiguration("available_points_ratio")},
                    {"max_z_distance": LaunchConfiguration("max_z_distance")},
                    {"num_track_points": LaunchConfiguration("num_track_points")},
                    {"displacements_outside": LaunchConfiguration("displacements_outside")},
                    {"displacements_inside": LaunchConfiguration("displacements_inside")},
                    {"displacements_near": LaunchConfiguration("displacements_near")},
                    {"displacements_far": LaunchConfiguration("displacements_far")},
                    {"realsense_simulation": LaunchConfiguration("realsense_simulation")},
                ],
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

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
                "foot_gap",
                default_value="0.25",
                description="Horizontal distance between the two feet",
            ),
            DeclareLaunchArgument(
                "step_distance",
                default_value="0.30",
                description="Step size",
            ),
            DeclareLaunchArgument(
                "outlier_distance",
                default_value="0.05",
                description="Distance metric used to classify points as outliers",
            ),
            DeclareLaunchArgument(
                "sample_size",
                default_value="3",
                description="Number of points used for temporal averages",
            ),
            DeclareLaunchArgument(
                "height_zero_threshold",
                default_value="0.02",
                description="Threshold for point heights to be non zero",
            ),
            DeclareLaunchArgument(
                "foot_width",
                default_value="0.10",
                description="Width of the rectangle used to find foot positions",
            ),
            DeclareLaunchArgument(
                "foot_length",
                default_value="0.20",
                description="Length of the rectangle used to find foot positions",
            ),
            DeclareLaunchArgument(
                "actual_foot_length",
                default_value="0.33",
                description="Length of the actual foot",
            ),
            DeclareLaunchArgument(
                "derivative_threshold",
                default_value="0.02",
                description="Maximum allowed second derivative values",
            ),
            DeclareLaunchArgument(
                "available_points_ratio",
                default_value="0.90",
                description="Ratio of points that should gave a derivative lower than the threshold",
            ),
            DeclareLaunchArgument(
                "max_z_distance",
                default_value="0.25",
                description="Maximum allowed height difference between two steps",
            ),
            DeclareLaunchArgument(
                "num_track_points",
                default_value="30",
                description="Number of points in the point track between start and end positions",
            ),
            DeclareLaunchArgument(
                "displacements_outside",
                default_value="0.05",
                description="Distance the algorithm looks outside of the optimal point",
            ),
            DeclareLaunchArgument(
                "displacements_inside",
                default_value="0.10",
                description="Distance the algorithm looks inside of the optimal point",
            ),
            DeclareLaunchArgument(
                "displacements_near",
                default_value="0.25",
                description="Distance the algorithm looks in front of the optimal point",
            ),
            DeclareLaunchArgument(
                "displacements_far",
                default_value="0.10",
                description="Distance the algorithm looks behind the optimal point",
            ),
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
                respawn=True,
                emulate_tty=True,
                arguments=[('__log_level:=debug')],
                parameters=[
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

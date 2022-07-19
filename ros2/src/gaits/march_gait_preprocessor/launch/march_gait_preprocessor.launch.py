"""Author: Marten  Haitjema, MVII."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
)

# Get lengths from urdf:
lengths = get_lengths_robot_from_urdf_for_inverse_kinematics()
LENGTH_HIP_AA, LENGTH_HIP_BASE = lengths[2], lengths[-1]
DEFAULT_FEET_DISTANCE = LENGTH_HIP_AA * 2 + LENGTH_HIP_BASE


def generate_launch_description():
    """Launch file for the gait_preprocessor node that will spam fake possible foot location.

    For more information see '../march_gait_preprocessor/gait_preprocessor_publisher.py'.

    Can change parameters during runtime by calling in a terminal:
        'ros2 param set gait_preprocessor_node [param_name] [value]'
        with param_name and value possibilities:
        * location_x: either a double or 'random'
        * location_y: either a double or 'random'
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            DeclareLaunchArgument(
                name="location_x",
                default_value="0.5",
                description="x-location for fake covid topic, takes double'",
            ),
            DeclareLaunchArgument(
                name="location_y",
                default_value="0.03",
                description="y-location for fake covid topic, takes double",
            ),
            DeclareLaunchArgument(
                name="duration",
                default_value="1.5",
                description="Base duration of dynamic gait, may be scaled depending on step height",
            ),
            DeclareLaunchArgument(
                name="location_z",
                default_value=str(DEFAULT_FEET_DISTANCE),
                description="z-location for fake covid topic, takes double or 'random'",
            ),
            DeclareLaunchArgument(
                name="deviation_coefficient",
                default_value="0.3",
                description="Coefficient used to compute the deviation of two midpoints from a middle fraction",
            ),
            DeclareLaunchArgument(
                name="midpoint_increase",
                default_value="0.1",
                description="How much higher midpoints should be based on the highest track point",
            ),
            DeclareLaunchArgument(
                name="minimum_high_point_ratio",
                default_value="0.1",
                description="Minimum ratio of high points before the midpoints deviation can become larger than 0",
            ),
            DeclareLaunchArgument(
                name="max_deviation",
                default_value="0.15",
                description="Maximum allowed midpoint deviation from the standard midpoint fraction",
            ),
            DeclareLaunchArgument(
                name="new_midpoint_method",
                default_value="True",
                description="Whether to use the new method of calculating mid points.",
            ),
            Node(
                package="march_gait_preprocessor",
                executable="march_gait_preprocessor",
                output="screen",
                name="gait_preprocessor",
                namespace="march",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"location_x": LaunchConfiguration("location_x")},
                    {"location_y": LaunchConfiguration("location_y")},
                    {"location_z": LaunchConfiguration("location_z")},
                    {"duration": LaunchConfiguration("duration")},
                    {"deviation_coefficient": LaunchConfiguration("deviation_coefficient")},
                    {"midpoint_increase": LaunchConfiguration("midpoint_increase")},
                    {"minimum_high_point_ratio": LaunchConfiguration("minimum_high_point_ratio")},
                    {"max_deviation": LaunchConfiguration("max_deviation")},
                    {"new_midpoint_method": LaunchConfiguration("new_midpoint_method")},
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

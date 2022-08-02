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
                default_value="true",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            DeclareLaunchArgument(
                name="location_x",
                default_value="0.4",
                description="x-location for fake covid topic, takes double'",
            ),
            DeclareLaunchArgument(
                name="location_y",
                default_value="0.0",
                description="y-location for fake covid topic, takes double",
            ),
            DeclareLaunchArgument(
                name="location_z",
                default_value=str(DEFAULT_FEET_DISTANCE),
                description="z-location for fake covid topic, takes double or 'random'",
            ),
            DeclareLaunchArgument(
                name="offset_x",
                default_value="0.07",
                description="X offset for covid points.",
            ),
            DeclareLaunchArgument(
                name="offset_y",
                default_value="0.0",
                description="Y offset for covid points.",
            ),
            DeclareLaunchArgument(
                name="offset_z",
                default_value="0.22",
                description="Z offset for covid points.",
            ),
            DeclareLaunchArgument(
                name="duration",
                default_value="1.3",
                description="Base duration of dynamic gait, may be scaled depending on step height",
            ),
            DeclareLaunchArgument(
                name="deviation_coefficient",
                default_value="0.5",
                description="Coefficient used to compute the deviation of two midpoints from a middle fraction",
            ),
            DeclareLaunchArgument(
                name="max_deviation",
                default_value="0.15",
                description="Maximum allowed midpoint deviation from the standard midpoint fraction",
            ),
            DeclareLaunchArgument(
                name="use_simulated_deviation",
                default_value="false",
                description="Whether to use simulated, fixed, deviation for calculating mid points.",
            ),
            DeclareLaunchArgument(
                name="simulated_deviation", default_value="0.05", description="midpoint deviation for simulated points."
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
                    {"offset_x": LaunchConfiguration("offset_x")},
                    {"offset_y": LaunchConfiguration("offset_y")},
                    {"offset_z": LaunchConfiguration("offset_z")},
                    {"duration": LaunchConfiguration("duration")},
                    {"deviation_coefficient": LaunchConfiguration("deviation_coefficient")},
                    {"max_deviation": LaunchConfiguration("max_deviation")},
                    {"use_simulated_deviation": LaunchConfiguration("use_simulated_deviation")},
                    {"simulated_deviation": LaunchConfiguration("simulated_deviation")},
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

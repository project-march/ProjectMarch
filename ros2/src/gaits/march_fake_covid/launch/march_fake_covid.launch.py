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
    """Launch file for the fake_covid_publisher node that will spam fake possible foot location.
    For more information see '../march_fake_covid/fake_covid_publisher.py'.

    Can change parameters during runtime by calling in a terminal:
        'ros2 param set fake_covid_publisher [param_name] [value]'
        with param_name and value possibilities:
        * location_x: either a double or 'random'
        * location_y: either a double or 'random'
    """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="location_x",
                default_value="0.4",
                description="x-location for fake covid topic, takes double or 'random'",
            ),
            DeclareLaunchArgument(
                name="location_y",
                default_value="0.0",
                description="y-location for fake covid topic, takes double or 'random'",
            ),
            DeclareLaunchArgument(
                name="location_z",
                default_value=str(DEFAULT_FEET_DISTANCE),
                description="z-location for fake covid topic, takes double or 'random'",
            ),
            Node(
                package="march_fake_covid",
                executable="march_fake_covid",
                output="screen",
                name="fake_covid_publisher",
                namespace="march",
                parameters=[
                    {"location_x": LaunchConfiguration("location_x")},
                    {"location_y": LaunchConfiguration("location_y")},
                    {"location_z": LaunchConfiguration("location_z")},
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

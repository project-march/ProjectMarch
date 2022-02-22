from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


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
                name="simulate_points",
                default_value="False",
                description="Whether to simulate fake foot positions for gait generation",
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
                name="duration",
                default_value="1.5",
                description="Base duration of dynamic gait, may be scaled depending on step height",
            ),
            Node(
                package="march_gait_preprocessor",
                executable="march_gait_preprocessor",
                output="screen",
                name="gait_preprocessor",
                namespace="march",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"simulate_points": LaunchConfiguration("simulate_points")},
                    {"location_x": LaunchConfiguration("location_x")},
                    {"location_y": LaunchConfiguration("location_y")},
                    {"duration": LaunchConfiguration("duration")},
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

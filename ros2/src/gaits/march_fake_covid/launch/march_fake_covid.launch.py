from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="location_x",
                default_value="0.4",
                description="x-location for fake covid topic",
            ),
            DeclareLaunchArgument(
                name="location_y",
                default_value="0.0",
                description="y-location for fake covid topic",
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
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

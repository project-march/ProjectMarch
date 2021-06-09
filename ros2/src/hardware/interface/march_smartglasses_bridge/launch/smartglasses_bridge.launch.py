from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="hud_host",
                default_value="localhost",
                description="The host address of the HUD",
            ),
            DeclareLaunchArgument(
                name="hud_port",
                default_value="53003",
                description="The port on which the HUD listens",
            ),
            Node(
                package="march_smartglasses_bridge",
                executable="march_smartglasses_bridge",
                namespace="march/smart_glasses",
                output="screen",
                arguments=[],
                parameters=[
                    {"hud_host": LaunchConfiguration("hud_host")},
                    {"hud_port": LaunchConfiguration("hud_port")},
                ],
            ),
        ]
    )

"""Author: Thijs Raymakers, MVI."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file configuration for the smart glasses.

    It starts the smart_glasses node under the name 'march_smartglasses_bridge'.

    The settable ros parameters are:
        hud_host (str): The Wi-Fi address where the smart glasses are on (excluding the port). Default is "local_host".
        hud_port (int): The port where the smart glasses are located. Default is `53003`.
    """
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

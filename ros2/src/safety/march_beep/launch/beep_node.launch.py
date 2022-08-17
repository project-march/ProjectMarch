"""Author: Jelmer de Wolde, MVII."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file used to start the march_beep node."""
    return LaunchDescription(
        [
            Node(package="march_beep", namespace="march", executable="beep_node", name="march_beep_node"),
        ]
    )

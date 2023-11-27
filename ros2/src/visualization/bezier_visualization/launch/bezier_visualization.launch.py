"""Author: MVIII."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the Bezier visualization nodes.

    These nodes are started when the  Bezier visualization has be run.
    """
    return LaunchDescription(
        [
            Node(
                package="bezier_visualization",
                executable="bezier_visualization_node",
                name="bezier_visualization",
            ),
        ]
    )

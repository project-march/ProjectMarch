"""Author: MVIII."""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch description of the state estimator.

    This node is started when the state estimator is run
    """
    return LaunchDescription(
        [
            Node(
                package="march_mode_machine",
                namespace="",
                executable="mode_machine_node",
                name="mode_machine",
            ),
        ]
    )

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
                package="march_input_device",
                namespace="",
                executable="march_input_device_node",
                name="input_device",
            ),
        ]
    )

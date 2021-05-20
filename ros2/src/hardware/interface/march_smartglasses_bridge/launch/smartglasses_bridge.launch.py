from launch import LaunchDescription
import os
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="march_smartglasses_bridge",
                executable="march_smartglasses_bridge",
                namespace="march/smart_glasses",
                output="screen",
                arguments=[],
                parameters=[],
            ),
        ]
    )

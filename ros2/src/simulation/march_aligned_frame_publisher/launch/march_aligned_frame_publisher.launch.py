"""Author: Tuhin das, MVII."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Todo: Add docstring."""
    return LaunchDescription(
        [
            Node(
                package="march_aligned_frame_publisher",
                executable="march_aligned_frame_publisher_node",
                name="march_aligned_frame_publisher",
                namespace="march",
                output="screen",
                respawn=True,
            ),
        ]
    )


if __name__ == "__main__":
    generate_launch_description()

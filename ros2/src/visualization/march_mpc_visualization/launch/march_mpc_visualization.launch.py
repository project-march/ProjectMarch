"""Author: Thijs Veen, MVI."""
import launch
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """Starts the mpc visualization node."""
    return launch.LaunchDescription(
        [
            Node(
                package="march_mpc_visualization",
                executable="march_mpc_visualization",
                name="march_mpc_visualization",
                namespace="march",
                output="screen",
            ),
        ]
    )

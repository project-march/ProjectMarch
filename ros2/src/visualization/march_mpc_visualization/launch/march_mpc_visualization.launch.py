import launch
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription(
        [
            Node(
                package="march_mpc_visualization",
                executable="march_mpc_visualization",
                name="march_mpc_visualization",
                output="screen",
            ),
        ]
    )

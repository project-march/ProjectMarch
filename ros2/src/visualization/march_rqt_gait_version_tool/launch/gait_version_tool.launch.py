"""Author: MVI."""
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """Launch file to launch rqt note taker."""
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Whether to use simulation time",
            ),
            Node(
                package="march_rqt_gait_version_tool",
                executable="gait_version_tool",
                output="screen",
                name="gait_version_tool",
                namespace="march",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )

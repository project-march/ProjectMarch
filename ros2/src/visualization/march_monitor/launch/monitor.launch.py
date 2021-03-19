import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Whether to use simulation time",
            ),
            DeclareLaunchArgument(
                name="perspective",
                default_value="full_monitor",
                description="Which perspective file to use.",
            ),
            Node(
                package="rqt_gui",
                executable="rqt_gui",
                namespace="march/monitor",
                output="screen",
                arguments=[
                    "--perspective-file",
                    [
                        PathJoinSubstitution(
                            [
                                get_package_share_directory("march_monitor"),
                                "config",
                                LaunchConfiguration("perspective"),
                            ]
                        ),
                        ".perspective",
                    ],
                ],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("march_rqt_robot_monitor"),
                        "launch",
                        "march_rqt_robot_monitor.launch.py",
                    )
                ),
                launch_arguments=[("rqt", "false")],
            ),
        ]
    )

"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, condition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    rosbags = LaunchConfiguration("rosbags", default="true")

    declared_arguments = [
        DeclareLaunchArgument(
            name="rosbags",
            default_value="true",
            description="Whether the rosbags should stored.",
            choices=["true", "false"],
        )
    ]

    return LaunchDescription(declared_arguments + [
        Node(
            package='march_loggers',
            executable='hardware_interface_logger_node',
            name='hardware_interface_logger_node',
            output='screen',
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_safety"),
                "launch",
                "march_safety.launch.py",
            )
        ),
        launch_arguments=[("simulation", "true")],
    ),
        ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "record",
                "-o",
                '~/rosbags2/$(date -d "today" +"%Y-%m-%d-%H-%M-%S")',
                "-a",
            ],
            output={
                "stdout": "log",
                "stderr": "log",
            },
            shell=True,  # noqa: S604 This is ran as shell so that -o data parsing and regex can work correctly.
            condition=IfCondition(rosbags),
        )
    ])
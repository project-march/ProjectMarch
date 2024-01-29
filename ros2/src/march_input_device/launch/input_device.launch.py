"""Author: MVIII."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument

def generate_launch_description():
    """Launch description of the input device.

    This node is started when the input device is run
    """
    IPD_new_terminal = LaunchConfiguration('IPD_new_terminal')
    arguments = [
        DeclareLaunchArgument(
            name="IPD_new_terminal",
            default_value="true",
            description="Whether a new terminal should be openened, allowing you to give input.",
        ),
    ]

    node_in_current_terminal = Node(
        package="march_input_device",
        namespace="",
        executable="input_device_node",
        name="input_device",
        parameters=[{"IPD_new_terminal": IPD_new_terminal}],
        condition=conditions.UnlessCondition(IPD_new_terminal)
    )

    node_in_new_terminal = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'run', 'march_input_device', 'input_device_node'],
        output='screen',
        condition=conditions.IfCondition(IPD_new_terminal)
    )

    return LaunchDescription(
        arguments + [node_in_current_terminal, node_in_new_terminal]
    )
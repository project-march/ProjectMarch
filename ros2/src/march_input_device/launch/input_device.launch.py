"""Author: MVIII."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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
    node = Node(
                package="march_input_device",
                namespace="",
                executable="input_device_node",
                name="input_device",
                parameters=[{"IPD_new_terminal": IPD_new_terminal}]
            ),
# # 
#     if IPD_new_terminal == 'true':
#         node = ExecuteProcess(
#             cmd=['gnome-terminal', '--', 'ros2', 'run', 'march_input_device', 'input_device_node'],
#             output='screen'
#         )    


    return LaunchDescription(
        [
            arguments,
            node
        ]
    )

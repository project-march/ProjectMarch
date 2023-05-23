from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()


    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    # arg2 = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(arg)
    # ld.add_action(arg2)

    joint_trajectory_buffer_node = Node(
        package='joint_trajectory_buffer',
        executable='joint_trajectory_buffer_node',
        name='joint_trajectory_buffer_node',
        # arguments=['--ros-args', '--log-level', 'debug']
    )
    ld.add_action(joint_trajectory_buffer_node)

    return ld

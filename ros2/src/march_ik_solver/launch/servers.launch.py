from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='march_ik_solver',
            executable='task_server',
            name='task_server',
        ),
        Node(
            package='march_ik_solver',
            executable='current_joint_positions_server',
            name='current_joint_positions_server',
        ),
    ])


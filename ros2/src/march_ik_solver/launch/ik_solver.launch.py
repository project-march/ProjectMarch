import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
                get_package_share_directory('march_ik_solver'),
                'config',
                'ik_solver.yaml'
            )

    return LaunchDescription([
        Node(
            package='march_ik_solver',
            # namespace='march_ik_solver',
            executable='ik_solver_node',
            name='ik_solver',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='march_ik_solver',
            # namespace='march_ik_solver',
            executable='ik_solver_buffer_node',
            name='ik_solver_buffer',
            output='screen',
            parameters=[config],
        ),
    ])
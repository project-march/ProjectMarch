import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
                get_package_share_directory('march_ik_solver'),
                'config',
                'ik_point_evaluation.yaml'
            )
    
    return LaunchDescription([
        Node(
            package='march_ik_solver',
            executable='ik_solver_point_evaluator_node',
            name='ik_solver_point_evaluator',
            output='screen',
            parameters=[config],
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='march_ik_solver',
            # namespace='march_ik_solver',
            executable='ik_solver_node',
            name='ik_solver',
            output='screen',
        ),
        # Node(
        #     package='march_ik_solver',
        #     # namespace='march_ik_solver',
        #     executable='ik_solver_buffer_node',
        #     name='ik_solver_buffer',
        #     output='screen',
        #     parameters=[
        #         {'dt': 1e-3},
        #         {'convergence_threshold': 0.5},
        #     ],
        # ),
    ])
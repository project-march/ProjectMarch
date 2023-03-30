"""Author: MVIII."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the ik solver.

    This node is started when the ik solver is ran
    """
    urdf_location = os.path.join(
        get_package_share_directory('march_description'),
        'urdf',
        'march7_FROST.urdf'
    )

    params = {'robot_description': urdf_location}
    return LaunchDescription([
        Node(
            package='ik_solver',
            namespace='',
            executable='ik_solver_node',
            name='ik_solver',
            parameters=[
                params
            ]
        ),
    ])

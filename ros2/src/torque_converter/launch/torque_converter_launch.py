"""Author: MVIII."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the torque converter.

    This node is started when the torque converter is ran
    """
    urdf_default = os.path.join(
        get_package_share_directory('march_description'),
        'urdf',
        'hennie_v0.urdf'
    )

    # parameters
    urdf_location = LaunchConfiguration("robot_description", default=urdf_default)
    timestep = LaunchConfiguration("timestep", default='8')

    params = {'robot_description': urdf_location}
    return LaunchDescription([
        Node(
            package='torque_converter',
            namespace='',
            executable='torque_converter_node',
            name='torque_converter',
            parameters=[
                params,
                {"timestep", timestep},
            ]
        ),
    ])
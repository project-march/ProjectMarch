import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the mujoco simulation nodes.

    These nodes are started when the mujoco simulation has be run.
    """
    config = os.path.join(
    get_package_share_directory('state_estimator'),
    'config',
    'state_estimation_setup_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='state_estimator',
            namespace='',
            executable='state_estimator_node',
            name='state_estimator',
            parameters=[
                config,
            ]
        ),
    ])

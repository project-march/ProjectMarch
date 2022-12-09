import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mujoco_sim'),
        'config',
        'low_level_controller_tunings.yaml'
    )

    return LaunchDescription([
        Node(
            package='mujoco_sim',
            namespace='',
            executable='mujoco_sim_node',
            name='mujoco_sim',
            parameters=[
                config,
                {"model_toload": "march.xml"}
            ]
        ),
        Node(
            package='mujoco_reader',
            namespace='',
            executable='mujoco_reader_node',
            name='mujoco_reader',
            parameters=[
                {"reader_data_type": 0}
            ]
        ),
        Node(
            package='mujoco_writer',
            executable='mujoco_writer_node',
            name='mujoco_writer',
        ),
    ])

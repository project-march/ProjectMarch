import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='march_state_estimator',
            executable='robot_description_node',
            name='robot_description',
            # parameters=[
            #     {
            #         'urdf_path': os.path.join(
            #                 get_package_share_directory('march_description'),
            #                 'urdf',
            #                 'march8',
            #                 'hennie_with_koen.urdf'
            #             )
            #     },
            # ]
        ),
    ])

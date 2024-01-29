import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # config = os.path.join(
    #     get_package_share_directory('march_state_estimator'),
    #     'config',
    #     'state_estimator_config.yaml'
    # )

    config = os.path.join(
        get_package_share_directory('march_state_estimator'),
        'config',
        'robot_definition-config.yaml'
    )

    return LaunchDescription([
        Node(
            package='march_state_estimator',
            executable='state_estimator_node',
            name='state_estimator',
            output='screen',
            # parameters=[config]
        ),
        
    ])
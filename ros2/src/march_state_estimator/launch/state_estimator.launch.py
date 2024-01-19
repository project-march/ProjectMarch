import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('march_state_estimator'), 'launch', 'robot_description_hennie.launch.py')]),
        ),
        Node(
            package='march_state_estimator',
            executable='sensor_fusion_node',
            name='sensor_fusion',
            output='screen',
        ),
    ])
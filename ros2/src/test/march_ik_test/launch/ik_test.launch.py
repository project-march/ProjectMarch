import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch file directory
    march_ik_test_dir = get_package_share_directory('march_ik_test')

    # Create the launch description and start the ik_test_node
    return LaunchDescription([
        Node(
            package='march_ik_test',
            executable='ik_test_node',
            name='ik_test_node',
            output='screen',
            parameters=[
                os.path.join(march_ik_test_dir, 'config', 'ik_test.yaml')
            ]
        )
    ])
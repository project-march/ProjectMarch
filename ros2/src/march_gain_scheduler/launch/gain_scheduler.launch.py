from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('march_gain_scheduler'),'config','stand_gains.yaml') # sets the default config file path 
    config_path = LaunchConfiguration('config_path', default=default_config)

    os.environ['RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED'] = '1'

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path',
            default_value=default_config,
            description='Path to the config file.'
        ),
        Node(
            package='march_gain_scheduler',
            executable='gain_scheduler_node',
            name='gain_scheduler',
            output='screen',
            parameters=[{'config_path': config_path}]
        ),
    ])
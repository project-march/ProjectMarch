import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        DeclareLaunchArgument(
            'ping_safety_node',
            default_value='True',
            description='Whether to ping the safety node'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Whether to use simulation time'),
        Node(
            package='march_rqt_input_device_ros2', executable='input_device', output='screen',
            name='input_device', arguments=[LaunchConfiguration('ping_safety_node'), LaunchConfiguration('use_sim_time')])
    ])

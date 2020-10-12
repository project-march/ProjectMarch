import os

import launch
from ament_index_python import get_package_share_directory
from launch import LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
# import march_rqt_input_device.launch.input_device_launch

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
        launch.actions.DeclareLaunchArgument(
            name='rqt_input',
            default_value='True',
            description='Launches the rqt input device.'),
        # Launch rqt input device if not rqt_input:=false is given as argument
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('march_rqt_input_device'),
                                              'launch', 'input_device.launch.py')),
            launch_arguments=[('node_prefix', LaunchConfiguration('node_prefix')),
                              ('ping_safety_node', LaunchConfiguration('ping_safety_node')),
                              ('use_sim_time', LaunchConfiguration('use_sim_time'))],
            condition=IfCondition(LaunchConfiguration('rqt_input')))
    ])



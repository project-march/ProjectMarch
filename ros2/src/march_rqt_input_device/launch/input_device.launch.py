import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

'''
Launch file to launch rqt input device.
@argument: use_sim_time, whether the node should use the simulation time as published on the /clock topic.
@argument: ping_safety_node, whether the node should regularly send an Alive message for the safety node.
'''
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
            package='march_rqt_input_device', executable='input_device', output='screen',
            name='input_device', arguments=[LaunchConfiguration('ping_safety_node'), LaunchConfiguration('use_sim_time')])
    ])
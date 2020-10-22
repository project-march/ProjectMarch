import os
import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    # General arguments
    node_prefix = LaunchConfiguration('node_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    # Input device arguments
    ping_safety_node = LaunchConfiguration('ping_safety_node')
    rqt_input = LaunchConfiguration('rqt_input')
    # Description arguments
    robot_state_publisher = LaunchConfiguration('robot_state_publisher')
    xacro_path = LaunchConfiguration('xacro_path')

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Whether to use simulation time'),
        DeclareLaunchArgument(
            'ping_safety_node',
            default_value='True',
            description='Whether to ping the safety node'),
        DeclareLaunchArgument(
            name='rqt_input',
            default_value='True',
            description='Launches the rqt input device.'),
        DeclareLaunchArgument(
            name='robot_state_publisher',
            default_value='True',
            description='Launch the robot state publisher.'),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=os.path.join(get_package_share_directory('march_description'), 'urdf', 'march4.xacro'),
            description='path to urdf.xacro file to publish'),
        # Launch rqt input device if not rqt_input:=false is given as argument
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_rqt_input_device'), 'launch', 'input_device.launch.py')),
            launch_arguments=[('node_prefix', node_prefix),
                              ('ping_safety_node', ping_safety_node),
                              ('use_sim_time', use_sim_time)],
            condition=IfCondition(rqt_input)),
        # Launch robot state publisher (from march_description) if not robot_state_publisher:=false
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_description'), 'launch', 'march_description.launch.py')),
            launch_arguments=[('ping_safety_node', xacro_path),
                              ('use_sim_time', use_sim_time)],
            condition=IfCondition(robot_state_publisher))
    ])

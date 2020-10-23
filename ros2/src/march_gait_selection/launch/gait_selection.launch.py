from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """ Basic launch file to launch the gait selection node """
    return LaunchDescription([
        DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Whether to use the simulation time as provided on the /clock topic.'),
        DeclareLaunchArgument(
            'gait_package',
            default_value='march_gait_files',
            description='The package where the gait files are located.'),
        DeclareLaunchArgument(
            'gait_directory',
            default_value='training-v',
            description='The directory where the gait files are located, relatice to the gait_package.'),

        Node(
            package='march_gait_selection', executable='march_gait_selection', output='screen',
            name='gait_selection', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                                               {'gait_package': LaunchConfiguration('gait_package')},
                                               {'gait_directory': LaunchConfiguration('gait_directory')}])
    ])

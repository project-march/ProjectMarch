from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """ Basic launch file to launch the robot information node """
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Whether to use simulation time as published on the '
                        '/clock topic by gazebo instead of system time.'),
        Node(
            package='march_robot_information',
            executable='march_robot_information',
            output='screen',
            name='robot_information',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])
    ])

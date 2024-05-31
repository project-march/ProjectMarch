import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    march_description_dir = get_package_share_directory('march_description')
    urdf_file = os.path.join(march_description_dir, 'urdf', 'march9', 'flywheel_pendulum', 'flywheel_pendulum.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_file = os.path.join(march_description_dir, 'rviz', 'flywheel_pendulum.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
            arguments=[urdf_file],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])
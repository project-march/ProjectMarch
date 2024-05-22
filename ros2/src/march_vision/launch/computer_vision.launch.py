import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config = os.path.join(
                get_package_share_directory('elevation_mapping'),
                'config',
                'gt_test.yaml'
            )

    return LaunchDescription([
        Node(
            package='elevation_mapping',
            executable='elevation_mapping',
            name='elevation_mapping',
            output='screen',
            parameters=[config],
        ),

        Node(
            package='elevation_mapping',
            executable="test_node_points",
            name="test_node_points",
        ),
    ])

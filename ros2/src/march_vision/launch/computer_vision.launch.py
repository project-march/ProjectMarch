import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    march_vision_config = os.path.join(
                get_package_share_directory('march_vision'),
                'config',
                'march_vision_config.yaml'
            )
    elevation_mapping_config = os.path.join(
            get_package_share_directory('march_vision'),
            'config',
            'elevation_mapping_config.yaml'
        )
    plane_segmentation_config = os.path.join(
            get_package_share_directory('march_vision'),
            'config',
            'plane_segmentation_config.yaml'
        )

    return LaunchDescription([
        Node(
            package='march_vision',
            executable='elevation_mapping_node',
            name='elevation_mapping',
            output='screen',
            parameters=[elevation_mapping_config],
        ),

        # Node(
        #     package='march_vision',
        #     executable='plane_segmentation_pipeline_node',
        #     name='plane_segmentation',
        #     output='screen',
        #     parameters=[plane_segmentation_config],
        # ),

        # Node(
        #     package='march_vision',
        #     executable='computer_vision_node',
        #     name='march_vision',
        #     output='screen',
        #     parameters=[march_vision_config],
        # ),
    ])

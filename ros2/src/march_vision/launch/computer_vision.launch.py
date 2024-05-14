import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    computer_vision_config = os.path.join(
                get_package_share_directory('march_vision'),
                'config',
                ## TODO: Change to correct file for launching with exo
                'gt_test.yaml'
            )

    return LaunchDescription([
        Node(
            package='march_vision',
            executable='elevation_mapping_node',
            name='elevation_mapping',
            output='screen',
            parameters=[computer_vision_config],
        ),

        Node(
            package='march_vision',
            executable='plane_segmentation_pipeline_node',
            name='plane_segmentation_pipeline',
            output='screen',
            parameters=[computer_vision_config],
        ),

        Node(
            package='march_vision',
            executable='computer_vision_node',
            name='computer_vision',
            output='screen',
            parameters=[computer_vision_config],
        ),
        # TODO: Add the test point cloud node 
        # For debugging and testing with recorded point clouds
        # Node(
        #     package='elevation_mapping',
        #     executable="test_node_points",
        #     name="test_node_points",
        # ),
    ])

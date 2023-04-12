#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_description = LaunchConfiguration("robot_description")
    xacro_path = PathJoinSubstitution(
        [
            get_package_share_directory("march_description"),
            "urdf",
            robot_description,
        ]
    )
    use_imu_data = LaunchConfiguration("use_imu_data")
    to_world_transform = LaunchConfiguration("to_world_transform")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                name="robot_description",
                default_value="march4",
                description="Which <robot_description>.xacro file to use. "
                "This file must be available in the march_desrciption/urdf/ folder",
            ),
            DeclareLaunchArgument(
                name="use_imu_data",
                default_value="False",
                description="Whether the data from the physical imu should be used to"
                "publish the rotation of the exoskeleton.",
            ),
            DeclareLaunchArgument(
                name="to_world_transform",
                default_value="False",
                description="Whether a transform from the world to base_link is "
                "necessary, this is the case when you are "
                "groundgaiting in rviz.",
            ),
            Node(
                package="march_robot_state_publisher",
                executable="march_robot_state_publisher_node",
                name="robot_state_publisher",
                namespace="march",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": Command(
                            ["xacro", " ", xacro_path, ".xacro"]
                        ),
                        "use_imu_data": use_imu_data,
                        "to_world_transform": to_world_transform,
                    }
                ],
            ),
        ]
    )

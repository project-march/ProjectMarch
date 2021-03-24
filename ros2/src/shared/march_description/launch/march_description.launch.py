#!/usr/bin/env python3
import xacro
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_description = LaunchConfiguration("robot_description")
    ground_gait = LaunchConfiguration("ground_gait")
    realsense_simulation = LaunchConfiguration("realsense_simulation")

    xacro_path = PathJoinSubstitution(
        [get_package_share_directory("march_description"), "urdf", robot_description]
    )
    use_imu_data = LaunchConfiguration("use_imu_data")
    imu_topic = LaunchConfiguration("imu_topic")
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
                name="imu_topic",
                default_value="/camera_front/imu/data",
                description="The topic that should be used to determine the orientation"
            ),
            DeclareLaunchArgument(
                name="to_world_transform",
                default_value="False",
                description="Whether a transform from the world to base_link is "
                "necessary, this is the case when you are "
                "groundgaiting in rviz.",
            ),
            DeclareLaunchArgument(
                name="simulation",
                default_value="False",
                description="Whether the exoskeleton is ran physically or in "
                "simulation.",
            ),
            DeclareLaunchArgument(
                name="realsense_simulation",
                default_value="False",
                description="Whether the simulation camera or the physical camera should be used",
            ),
            DeclareLaunchArgument(
                name="ground_gait",
                default_value="False",
                description="Whether the simulation should be simulating "
                "ground_gaiting instead of airgaiting.",
            ),
            DeclareLaunchArgument(
                "balance",
                default_value="False",
                description="Whether balance is being used.",
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
                            [
                                "xacro ",
                                xacro_path,
                                ".xacro",
                                " ground_gait:=",
                                ground_gait,
                                " realsense_simulation:=",
                                realsense_simulation,
                            ]
                        ),
                        "use_imu_data": use_imu_data,
                        "to_world_transform": to_world_transform,
                        "imu_topic": imu_topic
                    }
                ],
            ),
        ]
    )

#!/usr/bin/env python3
"""Script to load in the xacro and make it usable by ros."""
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """The launch file for the exo description.

    This makes sure the urdf is loaded in and published by the node `march_robot_state_publisher`.

    Todo:
        - Fill in the settable ros parameters.

    The settable ros parameters are:
        use_sim_time (bool): Whether the node should use the simulation time as published on the /clock topic.
            Default is false.
        ...
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = LaunchConfiguration("robot_description")
    ground_gait = LaunchConfiguration("ground_gait")
    realsense_simulation = LaunchConfiguration("realsense_simulation")
    jointless = LaunchConfiguration("jointless")

    xacro_path = PathJoinSubstitution([get_package_share_directory("march_description"), "urdf", robot_description])
    use_imu_data = LaunchConfiguration("use_imu_data")
    imu_topic = LaunchConfiguration("imu_topic")
    to_world_transform = LaunchConfiguration("to_world_transform")
    simulation = LaunchConfiguration("simulation")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
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
                description="The topic that should be used to determine the orientation",
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
                description="Whether the exoskeleton is ran physically or in simulation.",
            ),
            DeclareLaunchArgument(
                name="realsense_simulation",
                default_value="False",
                description="Whether the simulation camera or the physical camera should be used",
            ),
            DeclareLaunchArgument(
                name="ground_gait",
                default_value="False",
                description="Whether the simulation should be simulating ground_gaiting instead of airgaiting.",
            ),
            DeclareLaunchArgument(
                "balance",
                default_value="False",
                description="Whether balance is being used.",
            ),
            DeclareLaunchArgument(
                "jointless",
                default_value="False",
                description="If true, no joints will be actuated",
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
                                " configuration:=",
                                ("exoskeleton" if not simulation else "simulation"),
                                " jointless:=",
                                jointless,
                            ]
                        ),
                        "use_imu_data": use_imu_data,
                        "to_world_transform": to_world_transform,
                        "imu_topic": imu_topic,
                        "simulation": simulation,
                    }
                ],
            ),
        ]
    )

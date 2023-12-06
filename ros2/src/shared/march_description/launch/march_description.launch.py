#!/usr/bin/env python3
"""Script to load in the xacro and make it usable by ros."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
)
from launch_ros.actions import Node

from march_utility.utilities.build_tool_functions import generate_robot_desc_command


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
    # region XACRO arguments
    """Because the following are needed by xacro and should be the same as the ones given to control,
    they do not have a DeclaredLaunchArgument to ensure these are defined at a higher level that this launch file.
    """
    # Which <robot_description>.xacro file to use. This file must be available in the `march_description/urdf/` folder.
    robot_description = LaunchConfiguration("robot_description")
    # Whether the simulation should be simulating ground_gaiting instead of airgaiting.
    ground_gait = LaunchConfiguration("ground_gait")
    # Whether the simulation camera or the physical camera should be used.
    # realsense_simulation = LaunchConfiguration("realsense_simulation")
    # If true, no joints will be actuated.
    jointless = LaunchConfiguration("jointless")
    # Whether the exoskeleton is ran physically or in simulation.
    simulation = LaunchConfiguration("simulation")
    # Path to the control file. Must be in `march_control/config/`. Is not used if simulation:=true.
    gazebo_control_yaml = LaunchConfiguration("gazebo_control_yaml")
    # endregion

    # region March statepublisher arguments.
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_imu_data = LaunchConfiguration("use_imu_data")
    imu_topic = LaunchConfiguration("imu_topic")
    to_world_transform = LaunchConfiguration("to_world_transform")

    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
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
            description="The topic that should be used to determine the orientation.",
        ),
        DeclareLaunchArgument(
            name="to_world_transform",
            default_value="False",
            description="Whether a transform from the world to base_link is necessary, "
            "this is the case when you are groundgaiting in rviz.",
        ),
        DeclareLaunchArgument(
            "balance",
            default_value="False",
            description="Whether balance is being used.",
        ),
    ]
    # endregion

    march_state_publisher_node = Node(
        package="march_robot_state_publisher",
        executable="march_robot_state_publisher_node",
        name="robot_state_publisher",
        # Might be nice to add march namespace later on but /robot description should then be remapped.
        # namespace="march",  # noqa: E800
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    generate_robot_desc_command(
                        robot_descr_file=robot_description,
                        ground_gait=ground_gait,
                        # realsense_simulation=realsense_simulation,
                        simulation=simulation,
                        jointless=jointless,
                        gazebo_control_yaml=gazebo_control_yaml,
                    )
                ),
                "use_sim_time": use_sim_time,
                "use_imu_data": use_imu_data,
                "to_world_transform": to_world_transform,
                "imu_topic": imu_topic,
                "simulation": simulation,
            }
        ],
    )

    return LaunchDescription(declared_arguments + [march_state_publisher_node])

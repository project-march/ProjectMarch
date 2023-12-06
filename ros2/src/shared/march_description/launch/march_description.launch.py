#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
)
from launch_ros.actions import Node
from march_utility.utilities.build_tool_functions import generate_robot_desc_command


def generate_launch_description() -> LaunchDescription:
    # region Xacro arquments
    # Which <robot_description>.xacro file to use. This file must be available in the `march_description/urdf/` folder.
    robot_description = LaunchConfiguration("robot_description")
    # Whether the simulation should be simulating ground_gaiting instead of airgaiting.
    ground_gait = LaunchConfiguration("ground_gait")
    # If true, no joints will be actuated.
    jointless = LaunchConfiguration("jointless")
    # Whether the exoskeleton is ran physically or in simulation.
    simulation = LaunchConfiguration("simulation")
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
                        simulation=simulation,
                        jointless=jointless,
                    )
                ),
                "simulation": simulation,
            }
        ],
    )
    declared_arguments = []
    march_state_publisher_node = Node()
    return LaunchDescription(declared_arguments + [march_state_publisher_node])

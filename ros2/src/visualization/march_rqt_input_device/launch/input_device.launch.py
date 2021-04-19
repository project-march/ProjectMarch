import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """
    Launch file to launch rqt input device.
    :argument: use_sim_time, whether the node should use the simulation time as published on the /clock topic.
    :argument: ping_safety_node, whether the node should regularly send an Alive message for the safety node.
    """
    layout_file = [
        PathJoinSubstitution(
            [
                get_package_share_directory("march_rqt_input_device"),
                "config",
                LaunchConfiguration("layout"),
            ]
        ),
        ".json",
    ]
    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                name="node_prefix",
                default_value=[EnvironmentVariable("USER"), "_"],
                description="Prefix for node names",
            ),
            DeclareLaunchArgument(
                name="ping_safety_node",
                default_value="True",
                description="Whether to ping the safety node",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="False",
                description="Whether to use simulation time",
            ),
            DeclareLaunchArgument(
                name="layout",
                default_value="default",
                description="Layout .json file to use. Must be in the config directory.",
            ),
            Node(
                package="march_rqt_input_device",
                executable="input_device",
                output="screen",
                name="input_device",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"ping_safety_node": LaunchConfiguration("ping_safety_node")},
                    {"layout_file": layout_file},
                ],
            ),
        ]
    )

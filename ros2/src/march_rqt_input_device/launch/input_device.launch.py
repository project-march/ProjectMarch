"""Author: Olav de Haas, MIV; MVI & Andrew Hutani, MIX"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file to launch rqt input device.

    The settable ros parameters are:
        use_sim_time (bool): Whether the node should use the simulation time as published on the /clock topic.
        ping_safety_node (bool): Whether the node should regularly send an Alive message for the safety node.
    """
    return LaunchDescription(
        [
            Node(
                package="march_rqt_input_device",
                executable="input_device",
                name="input_device",
            ),
        ]
    )

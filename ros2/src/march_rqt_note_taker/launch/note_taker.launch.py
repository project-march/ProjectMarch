import launch
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """
    Launch file to launch rqt input device.
    :argument: use_sim_time, whether the node should use the simulation time as published on the /clock topic.
    :argument: ping_safety_node, whether the node should regularly send an Alive message for the safety node.
    """

    return launch.LaunchDescription([
        Node(package='march_rqt_note_taker', executable='note_taker', output='screen', name='note_taker')
    ])

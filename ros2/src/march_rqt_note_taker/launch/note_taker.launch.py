import launch
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    """
    Launch file to launch rqt note taker.
    """

    return launch.LaunchDescription([
        Node(package='march_rqt_note_taker', executable='note_taker', output='screen', name='note_taker')
    ])

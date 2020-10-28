import os
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='march_monitor',
            output='screen',

            ## Update perspective file to full_monitor.perspective when done with all monitor packages
            arguments=['--perspective-file', PathJoinSubstitution([get_package_share_directory('march_monitor'),
                                                                     'config', 'rqt_note_taker.perspective'])]
        ),
        # Uncomment this when march_rqt_robot_monitor is added
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #         os.path.join(get_package_share_directory('march_rqt_robot_monitor'), 'launch',
        #                      'march_rqt_robot_monitor.launch.py')),
        #     launch_arguments=[('rqt', 'false')]
        # )
    ])


if __name__ == '__main__':
    generate_launch_description()

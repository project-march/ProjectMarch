"""Author: MVIII."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the state estimator.

    This node is started when the state estimator is run
    """

    test1 = LaunchConfiguration("test1", default='1')
    test2 = LaunchConfiguration("test2", default='8.0')

    return LaunchDescription([
        Node(
        package='weight_shift_buffer',
        namespace='',
        executable='weight_shift_buffer_node',
        name='weight_shift_buffer_node',
        parameters=[
            {"test1": test1},
            {"test2": test2},
        ]
        ),
    ])

"""Author: MVIII."""
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch description of the state estimator.

    This node is started when the state estimator is run
    """

    test3 = LaunchConfiguration("weight_shift_type", default='1')
    test1 = LaunchConfiguration("weight_shift_duration", default='0.5')
    test2 = LaunchConfiguration("weight_shift_length", default='0.1')

    return LaunchDescription([
        Node(
        package='weight_shift_buffer',
        namespace='',
        executable='weight_shift_buffer_node',
        name='weight_shift_buffer_node',
        parameters=[
            {"weight_shift_duration": test1},
            {"weight_shift_length": test2},
            {"weight_shift_type": test3},
        ]
        ),
    ])

"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    test_rotational = LaunchConfiguration("test_rotational", default='true')

    # region Launch march control
    march_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_control"),
                "launch",
                "control_test_setup.launch.py",
            )
        ),
        launch_arguments=[
            ("test_rotational", test_rotational)
        ],
    )
    # endregion

    # region Launch rqt input device if not rqt_input:=false
    rqt_input_device = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_rqt_input_device"),
                "launch",
                "input_device.launch.py",
            )
        ),
        launch_arguments=[
            ("ping_safety_node", "true"),
            ("use_sim_time", "false"),
            ("layout", "test_joint"),
            ("testing", "true"),
        ],
    )

    return LaunchDescription([
        Node(
            package='state_machine',
            namespace='',
            executable='state_machine_node',
            name='state_machine',
        ),
        Node(
            package='test_setup_gait_selection',
            namespace='',
            executable='test_setup_gait_selection_node',
            name='test_setup_gait_selection',
            parameters=[
                {"test_rotational": test_rotational}
            ],
        ),
        rqt_input_device,
        march_control,
    ])

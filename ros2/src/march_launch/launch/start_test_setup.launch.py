"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    test_rotational = LaunchConfiguration("test_rotational", default='false')

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
    torque_converter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("torque_converter"),
                "launch",
                "torque_converter_launch.py",
            )
        )
    )
    # endregion


    # region Launch torque converter
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
    # endregion

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
        Node(
            package='fuzzy_generator',
            executable='fuzzy_node',
            name='fuzzy_node',
            # arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='joint_trajectory_buffer',
            executable='joint_trajectory_buffer_node',
            name='joint_trajectory_buffer_node',
            # arguments=['--ros-args', '--log-level', 'debug']
        ),
        # torque_converter,
        rqt_input_device,
        march_control,
    ])

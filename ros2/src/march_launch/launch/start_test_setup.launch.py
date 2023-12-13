"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    test_rotational = LaunchConfiguration("test_rotational", default=True)
    IPD_new_terminal = LaunchConfiguration("IPD_new_terminal", default="true")
    
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

    # region Launch input device
    input_device = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_input_device"),
                "launch",
                "input_device.launch.py",
            )
        ),
        launch_arguments=[
            ("IPD_new_terminal", IPD_new_terminal)
        ],
    )
    # endregion

    rosbags = LaunchConfiguration("rosbags", default='true')

    DeclareLaunchArgument(
        name="rosbags",
        default_value="true",
        description="Whether the rosbags should stored.",
        choices=["true", "false"],
    )

    # region rosbags
    # Make sure you have build the ros bags from the library not the ones from foxy!
    record_rosbags_action = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            '~/rosbags2/$(date -d "today" +"%Y-%m-%d-%H-%M-%S")',
            "-a",
        ],
        output={
            "stdout": "log",
            "stderr": "log",
        },
        shell=True,  # noqa: S604 This is ran as shell so that -o data parsing and regex can work correctly.
        condition=IfCondition(rosbags),
    )

    default_fuzzy_config = os.path.join(
        get_package_share_directory('fuzzy_generator'),
        'config',
        'joints.yaml'
    )

    # parameters
    fuzzy_config_path = LaunchConfiguration("config_path", default=default_fuzzy_config)

    return LaunchDescription([
        Node(
            package='state_machine',
            namespace='',
            executable='state_machine_node',
            name='state_machine',
        ),
        Node(
            package='march_gait_planning',
            namespace='',
            executable='test_gait_planning_node',
            name='test_gait_planning',
            parameters=[
                {"test_rotational": test_rotational}
            ],
        ),
        Node(
            package='fuzzy_generator',
            executable='fuzzy_node',
            name='fuzzy_node',
            parameters=[{'config_path': fuzzy_config_path}]
        ),
        input_device,
        march_control,
        record_rosbags_action,
    ])

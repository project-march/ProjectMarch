"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    test_rotational = LaunchConfiguration("test_rotational", default="true")
    IPD_new_terminal = LaunchConfiguration("IPD_new_terminal", default="true")
    
    # region Launch march control

    joint_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("march_control").find("march_control"),
                "launch",
                "control_multiple_slaves.launch.py",
            )
        ),
        launch_arguments=[
            ("test_rotational", test_rotational)
        ],
    )

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
    
    default_gainscheduler_config = os.path.join(
        get_package_share_directory('march_gain_scheduler'),
        'config',
        'stand_gains.yaml'
    )    

    # parameters
    fuzzy_config_path = LaunchConfiguration("config_path", default=default_fuzzy_config)
    gainscheduler_config_path = LaunchConfiguration("config_path", default=default_gainscheduler_config)

    return LaunchDescription([
        Node(
            package='march_mode_machine',
            namespace='',
            executable='test_joints_mode_machine_node',
            name='mode_machine',
        ),
        Node(
            package='march_gait_planning',
            namespace='',
            executable='test_setup_gait_planning_node',
            name='test_setup_gait_planning',
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
        Node(
            package='march_gain_scheduler',
            executable='gain_scheduler_node',
            name='gain_scheduler',
            parameters=[{'config_path': gainscheduler_config_path}]
        ),

        input_device,
        joint_launch_file,
        record_rosbags_action,
    ])

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
    # IPD_new_terminal = LaunchConfiguration("IPD_new_terminal", default="true")
    
    # region Launch march control
    rotational_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("march_control").find("march_control"),
                "launch",
                "rotational_control.launch.py",
            )
        ),
        launch_arguments=[
            ("test_rotational", test_rotational)
        ],
        condition=IfCondition(test_rotational),
    )

    linear_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("march_control").find("march_control"),
                "launch",
                "linear_control.launch.py",
            )
        ),
        launch_arguments=[
            ("test_rotational", test_rotational)
        ],
        condition=UnlessCondition(test_rotational),
    )
    # endregion
    # march_control = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("march_control"),
    #             "launch",
    #             "control_test_setup.launch.py",
    #         )
    #     ),
    #     launch_arguments=[
    #         ("test_rotational", test_rotational)
    #     ],
    # )
    

    # region Launch input device
    # input_device = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("march_input_device"),
    #             "launch",
    #             "input_device.launch.py",
    #         )
    #     ),
    #     launch_arguments=[
    #         ("IPD_new_terminal", IPD_new_terminal)
    #     ],
    # )
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

    default_gainscheduler_config = os.path.join(
        get_package_share_directory('march_gain_scheduler'),
        'config',
        'stand_gains.yaml'
    )    

    # parameters
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
            package='march_gain_scheduler',
            executable='gain_scheduler_node',
            name='gain_scheduler',
            parameters=[{'config_path': gainscheduler_config_path}]
        ),
        Node(
            package='march_test_joints_gui',
            executable='test_joints_gui_node',
            name='test_joints_gui_node',
        ),

        # input_device,
        rotational_launch_file,
        linear_launch_file,
        record_rosbags_action,
    ])

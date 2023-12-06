"""Author: MARCH IX"""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
)


def generate_launch_description() -> LaunchDescription:
    """Generates the default launch file for the exo.

    Todo:
        - Fill in the settable ros parameters.

    Implemented launch files:
        - "[march_rqt_input_device]/launch/input_device.launch.py"
        - "[march_description]/launch/march_description.launch.py"
        - "[march_gait_selection]/launch/gait_selection.launch.py"
        - "[march_fake_covid]/launch/march_fake_covid.launch.py"
        - "[march_safety]/launch/march_safety.launch.py"
        - "[march_robot_information]/launch/robot_information.launch.py"

    The settable ros parameters are:
        use_sim_time (bool): Whether the node should use the simulation time as published on the /clock topic.
            Default is True.
        wireless_ipd (bool): Whether the wireless IPD connection manager node should be started.
            Default is False.
        ...
    """

    # region LaunchConfigurations

    declared_arguments = []

    # General arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot = LaunchConfiguration("robot")
    rosbags = LaunchConfiguration("rosbags")
    rviz = LaunchConfiguration("rviz")

    # Input device arguments

    # TODO: Add input device arguments based on new IPD launch file

    # Robot state publisher arguments
    robot_state_publisher = LaunchConfiguration("robot_state_publisher")
    robot_description = LaunchConfiguration("robot_description")
    use_imu_data = LaunchConfiguration("use_imu_data")
    imu_topic = LaunchConfiguration("imu_topic")
    simulation = LaunchConfiguration("simulation")
    jointless = LaunchConfiguration("jointless")

    # RealSense/simulation argument
    # TODO: Change gait launch based on new gait launch file
    ground_gait = LaunchConfiguration("ground_gait")

    # endregion

    # region Simulation arguments
    ground_gait = LaunchConfiguration("ground_gait")
    to_world_transform = LaunchConfiguration("to_world_transform")
    mujoco = LaunchConfiguration("mujoco")
    mujoco_toload = LaunchConfiguration("model_to_load_mujoco", default='march8_v0.xml')
    tunings_to_load = LaunchConfiguration('tunings_to_load', default='low_level_controller_tunings.yaml')
    simulation_arguments = [
        DeclareLaunchArgument(
            name="ground_gait",
            default_value="false",
            description="Whether the simulation should be simulating ground_gaiting instead of airgaiting.",
        ),
        DeclareLaunchArgument(
            name="to_world_transform",
            default_value=ground_gait,
            description="Whether a transform from the world to base_link is necessary, "
            "this is the case when you are groundgaiting.",
        ),
        DeclareLaunchArgument(
            name="mujoco",
            default_value="false",
            description="If Mujoco simulation should be started or not",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            name="model_to_load_mujoco",
            default_value="march8_v0.xml",
            description="What model mujoco should load",
        ),
    ]
    # endregion

# Gait selection arguments
#TODO: Change gait launch based on new gait launch file

    declared_arguments = simulation_arguments + [
        # GENERAL ARGUMENTS
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Whether to use simulation time as published on the "
            "/clock topic by gazebo instead of system time.",
        ),
        DeclareLaunchArgument(name="robot", default_value="march7", description="Robot to use."),
        DeclareLaunchArgument(
            name="control_yaml",
            default_value="effort_control/march7_control.yaml",
            description="The controller yaml file to use loaded in through the controller manager "
            "(not used if gazebo control is used). Must be in: `march_control/config/`.",
        ),
        DeclareLaunchArgument(
            name="gazebo_control_yaml",
            default_value="gazebo/march7_control.yaml",
            description="The gazebo controller yaml file to use this is added in through the urdf published "
            "on /robot_description. Must be in: `march_control/config/`.",
        ),
        DeclareLaunchArgument(
            name="rosbags",
            default_value="true",
            description="Whether the rosbags should stored.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            name="rviz", default_value="false", description="Whether we should startup rviz.", choices=["true", "false"]
        ),
        # RQT INPUT DEVICE ARGUMENTS
        # TODO: Add RQT input based on new IPD launch file

        # ROBOT STATE PUBLISHER ARGUMENTS
        DeclareLaunchArgument(
            name="robot_state_publisher",
            default_value="true",
            description="Whether or not to launch the robot state publisher,"
            "this allows nodes to get the urdf and to subscribe to"
            "potential urdf updates. This is necesary for gait selection"
            "to be able to launch",
        ),
        DeclareLaunchArgument(
            name="robot_description",
            default_value=robot,
            description="Which <robot_description>.xacro file to use. "
            "This file must be available in the march_desrciption/urdf/ folder",
        ),
        DeclareLaunchArgument(
            "jointless",
            default_value="false",
            description="If true, no joints will be actuated",
        ),
        DeclareLaunchArgument(
            name="simulation",
            default_value="false",
            description="Whether the exoskeleton is ran physically or in simulation.",
        ),
        # GAIT SELECTION ARGUMENTS
        # TODO: Change gait selection arguments based on new gait launch file

    ]

    # region Launch robot description publisher (from march_description) if not robot_state_publisher:=false
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_description"),
                "launch",
                "march_description.launch.py",
            )
        ),
        launch_arguments=[
            ("robot_description", robot_description),
            ("use_sim_time", use_sim_time),
            ("ground_gait", ground_gait),
            ("to_world_transform", to_world_transform),
            ("use_imu_data", use_imu_data),
            ("imu_topic", imu_topic),
            ("simulation", simulation),
            ("jointless", jointless),
        ],
        condition=IfCondition(robot_state_publisher),
    )
    # endregion

    # region Launch March gait selection
    march_gait_selection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(   
                get_package_share_directory("march_gait_selection"),
                "launch",
                "gait_selection.launch.py",
            )
        ),
        launch_arguments=[
            ("gait_directory", gait_directory),
            ("use_sim_time", use_sim_time),
            ("gait_package", gait_package),
            ("balance", balance),
            ("dynamic_gait", dynamic_gait),
            ("middle_point_fraction", middle_point_fraction),
            ("middle_point_height", middle_point_height),
            ("minimum_stair_height", minimum_stair_height),
            ("push_off_fraction", push_off_fraction),
            ("push_off_position", push_off_position),
            ("add_push_off", add_push_off),
            ("amount_of_steps", amount_of_steps),
            ("use_position_queue", use_position_queue),
            ("add_cybathlon_gaits", add_cybathlon_gaits),
            ("scheduling_delay", scheduling_delay),
            ("first_subgait_delay", first_subgait_delay),
            ("timer_period", timer_period),
        ],
    )
    # endregion

    # region Launch Gait preprocessor
    gait_preprocessor_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_gait_preprocessor"),
                "launch",
                "march_gait_preprocessor.launch.py",
            )
        ),
    )
    # endregion

    # region Launch Safety
    safety_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_safety"),
                "launch",
                "march_safety.launch.py",
            )
        ),
        launch_arguments=[("use_sim_time", use_sim_time), ("simulation", simulation)],
    )
    # endregion

    # region Launch March robot information
    robot_information_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_robot_information"),
                "launch",
                "robot_information.launch.py",
            )
        )
    )
    # endregion

    # region Launch Gazebo
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("march_simulation"), "launch", "gazebo.launch.py"])]
        ),
        condition=IfCondition(gazebo),
    )
    # endregion

    # region Launch Mujoco
    mujoco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("mujoco_sim"), "mujoco_sim.launch.py"])]
        ),
        launch_arguments=[("model_to_load", mujoco_toload), ("tunings_to_load_path", PathJoinSubstitution([get_package_share_directory('march_control'), 'config', 'mujoco', tunings_to_load]))],
        condition=IfCondition(mujoco),
    )
    # endregion

    # region Launch march control
    march_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_control"),
                "launch",
                "controllers.launch.py",
            )
        ),
        launch_arguments=[("simulation", "true"), ("control_yaml", "mujoco/march7_control.yaml"), ("rviz", rviz)],
    )
    # endregion

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
            "-x",
            "'.*camera_.*'",
        ],
        output={
            "stdout": "log",
            "stderr": "log",
        },
        shell=True,  # noqa: S604 This is ran as shell so that -o data parsing and regex can work correctly.
        condition=IfCondition(rosbags),
    )
    # endregion

    imu_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bluespace_ai_xsens_mti_driver"),
                "launch",
                "imu_launch.launch.py",
            )
        ),
    )

    nodes = [
    rqt_input_device,
    wireless_ipd_node,
    robot_state_publisher_node,
    march_gait_selection_node,
    gait_preprocessor_node,
    safety_node,
    robot_information_node,
    mujoco_node,
    march_control,
    record_rosbags_action,
    ]


    return LaunchDescription(declared_arguments + nodes)

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
    mujoco_toload = LaunchConfiguration("model_to_load_mujoco", default='march8_v0.xml')
    tunings_to_load = LaunchConfiguration('tunings_to_load', default='low_level_controller_tunings.yaml')
    simulation = LaunchConfiguration("simulation", default='true')
    rosbags = LaunchConfiguration("rosbags", default='true')

    # Gait selection arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gait_package = LaunchConfiguration("gait_package")
    gait_directory = LaunchConfiguration("gait_directory")
    balance = LaunchConfiguration("balance")
    dynamic_gait = LaunchConfiguration("dynamic_gait")
    middle_point_fraction = LaunchConfiguration("middle_point_fraction")
    middle_point_height = LaunchConfiguration("middle_point_height")
    minimum_stair_height = LaunchConfiguration("minimum_stair_height")
    push_off_fraction = LaunchConfiguration("push_off_fraction")
    push_off_position = LaunchConfiguration("push_off_position")
    add_push_off = LaunchConfiguration("add_push_off")
    use_position_queue = LaunchConfiguration("use_position_queue")
    add_cybathlon_gaits = LaunchConfiguration("add_cybathlon_gaits")
    amount_of_steps = LaunchConfiguration("amount_of_steps")
    first_subgait_delay = LaunchConfiguration("first_subgait_delay")
    scheduling_delay = LaunchConfiguration("scheduling_delay")
    timer_period = LaunchConfiguration("timer_period")

    # GAIT SELECTION ARGUMENTS
    declared_arguments = [
        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Whether to use simulation time as published on the "
                        "/clock topic by gazebo instead of system time.",
        ),
        DeclareLaunchArgument(
            name="gait_package",
            default_value="march_gait_files",
            description="The package where the gait files are located.",
        ),
        DeclareLaunchArgument(
            name="gait_directory",
            default_value="airgait_vi",
            description="The directory in which the gait files to use are located, " "relative to the gait_package.",
        ),
        DeclareLaunchArgument(
            name="balance",
            default_value="false",
            description="Whether balance is being used.",
        ),
        DeclareLaunchArgument(
            name="dynamic_gait",
            default_value="true",
            description="Whether dynamic_setpoint_gait is enabled",
        ),
        DeclareLaunchArgument(
            name="middle_point_fraction",
            default_value="0.5",
            description="Fraction of the step at which the middle point of the dynamic gait will take place.",
        ),
        DeclareLaunchArgument(
            name="middle_point_height",
            default_value="0.15",
            description="Height of the middle setpoint of dynamic gait "
                        "relative to the desired position, given in meters.",
        ),
        DeclareLaunchArgument(
            name="minimum_stair_height",
            default_value="0.15",
            description="A step lower or higher than the minimum_stair_height"
                        "will change the gait type to stairs_like instead of walk_like.",
        ),
        DeclareLaunchArgument(
            name="push_off_fraction",
            default_value="0.15",
            description="Fraction of the step at which the push off will take place.",
        ),
        DeclareLaunchArgument(
            name="push_off_position",
            default_value="-0.15",
            description="Maximum joint position of the ankle during push off.",
        ),
        DeclareLaunchArgument(
            name="add_push_off",
            default_value="false",
            description="Whether to add a push off setpoint for the ankle.",
        ),
        DeclareLaunchArgument(
            name="amount_of_steps",
            default_value="0",
            description="Amount of steps the dynamic gait should make before stopping. 0 or -1 is infinite.",
        ),
        DeclareLaunchArgument(
            name="use_position_queue",
            default_value="false",
            description="Uses the values in position_queue.yaml for the half step if True, otherwise uses "
                        "points given by (simulated) covid.",
        ),
        DeclareLaunchArgument(
            name="add_cybathlon_gaits",
            default_value="false",
            description="Will add gaits created specifically for cybathlon obstacles to gait selection.",
        ),
        DeclareLaunchArgument(
            name="first_subgait_delay",
            default_value="0.0",
            description="Duration to wait before starting first subgait."
                        "If 0 then the first subgait is started immediately,"
                        "dropping the first setpoint in the process.",
        ),
        DeclareLaunchArgument(
            name="scheduling_delay",
            default_value="1.0",
            description="Duration to schedule next subgait early. If 0 then the"
                        "next subgait is never scheduled early.",
        ),
        DeclareLaunchArgument(name="timer_period", default_value="0.004", description=""),


        DeclareLaunchArgument(
            name="rosbags",
            default_value="true",
            description="Whether the rosbags should stored.",
            choices=["true", "false"],
        )
    ]
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

    # region Launch Mujoco
    mujoco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("mujoco_sim"), "mujoco_sim.launch.py"])]
        ),
        launch_arguments=[("model_to_load", mujoco_toload), ("tunings_to_load_path", PathJoinSubstitution(
            [get_package_share_directory('march_control'), 'config', 'mujoco', tunings_to_load]))],
        condition=IfCondition(simulation),
    )
    # endregion

    # region Launch march control
    march_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_control"),
                "launch",
                "march8_controllers.launch.py",
            )
        ),
        launch_arguments=[("simulation", simulation)],
    )
    # endregion

    # region rqt input device
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
            ("layout", "training"),
            ("testing", "false"),
        ],
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
        launch_arguments=[("simulation", "true")],
    )
    # endregion

    # region Launch Safety
    weight_shift_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("weight_shift_buffer"),
                "launch",
                "launch",
                "weight_shift_buffer_launch.py",
            )
        ),
        launch_arguments=[("simulation", "true")],
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
        ],
        output={
            "stdout": "log",
            "stderr": "log",
        },
        shell=True,  # noqa: S604 This is ran as shell so that -o data parsing and regex can work correctly.
        condition=IfCondition(rosbags),
    )
    # endregion

    state_estimator_launch_dir = os.path.join(
        get_package_share_directory('state_estimator'),
        'launch'
    )

    fuzzy_default_config = os.path.join(
        get_package_share_directory('fuzzy_generator'),
        'config',
        'joints.yaml'
    )

    # parameters
    fuzzy_config_path = LaunchConfiguration("config_path", default=fuzzy_default_config)

    return LaunchDescription(declared_arguments + [
        Node(
            package='fuzzy_generator',
            namespace='',
            executable='fuzzy_node',
            name='fuzzy_generator',
            parameters=[{'config_path': fuzzy_config_path}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_estimator_launch_dir, '/state_estimator_launch.py']),
        ),
        mujoco_node,
        rqt_input_device,
        march_control,
        record_rosbags_action,
        safety_node,
        gait_preprocessor_node,
        march_gait_selection_node,
        weight_shift_node,
    ])

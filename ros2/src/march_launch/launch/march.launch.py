"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, condition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    mujoco_toload = LaunchConfiguration("model_to_load_mujoco", default="march8_v0.xml")
    tunings_to_load = LaunchConfiguration("tunings_to_load", default="low_level_controller_tunings.yaml")
    simulation = LaunchConfiguration("simulation", default="true")
    rosbags = LaunchConfiguration("rosbags", default="true")
    airgait = LaunchConfiguration("airgait", default="false")
    robot = LaunchConfiguration("robot")
    IPD_new_terminal = LaunchConfiguration("IPD_new_terminal")

    DeclareLaunchArgument(
        name="rosbags",
        default_value="true",
        description="Whether the rosbags should stored.",
        choices=["true", "false"],
    )

    DeclareLaunchArgument(
        name="airgait",
        default_value="false",
        description="Whether we want to do an airgait or not",
        choices=["true", "false"],
    )

    DeclareLaunchArgument(
        name="robot",
        default_value="march8",
        description="The name of the yaml that will be used for retrieving info about the exo.",
    )

    DeclareLaunchArgument(
        name="IPD_new_terminal",
        default_value="true",
        description="Whether a new terminal should be openened, allowing you to give input.",
    )

    # region Launch Mujoco
    mujoco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("mujoco_sim"), "mujoco_sim.launch.py"])]),
        launch_arguments=[
            ("model_to_load", mujoco_toload),
            (
                "tunings_to_load_path",
                PathJoinSubstitution(
                    [get_package_share_directory("march_control"), "config", "mujoco", tunings_to_load]
                ),
            ),
        ],
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

    #TODO: implement own input device M9 

    # region rqt input device
    # rqt_input_device = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("march_rqt_input_device"),
    #             "launch",
    #             "input_device.launch.py",
    #         )
    #     ),
    #     launch_arguments=[
    #         ("ping_safety_node", "true"),
    #         ("use_sim_time", "false"),
    #         ("layout", "training"),
    #         ("testing", "false"),
    #     ],
    # )
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

    # region Launch IMU
    imu_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bluespace_ai_xsens_mti_driver"),
                "launch",
                "imu_launch.launch.py",
            )
        ),
    )
    # endregion

    # region Launch IPD
    ipd_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_input_device"),
                "launch",
                "input_device.launch.py",
            )
        ),
    ),
    #endregion



    # region Launch State Estimator
    state_estimator_launch_dir = os.path.join(get_package_share_directory("state_estimator"), "launch")

    state_estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([state_estimator_launch_dir, '/state_estimator.launch.py']),
        condition=UnlessCondition(airgait),
        ),
    # endregion

    # region Launch IK Solver
    ik_solver_launch_dir = os.path.join(get_package_share_directory("march_ik_solver"), "launch")
    urdf_location = os.path.join(
        get_package_share_directory("march_description"), "urdf", "march8", "hennie_with_koen.urdf"
    )
    # declare parameters
    # in ms
    trajectory_dt = 50

    ik_solver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ik_solver_launch_dir, '/ik_solver.launch.py']),
        launch_arguments={'robot_description': urdf_location, "timestep": str(trajectory_dt)}.items(),
    ),
    # endregion



    fuzzy_default_config = os.path.join(get_package_share_directory("fuzzy_generator"), "config", "joints.yaml")

    # parameters
    fuzzy_config_path = LaunchConfiguration("config_path", default=fuzzy_default_config)

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


    # region footstep_generation parameters
    # n_footsteps = 20
    # step_length = 0.2
    # endregion

    return LaunchDescription([
        Node(
            package='fuzzy_generator',
            namespace='',
            executable='fuzzy_node',
            name='fuzzy_generator',
            parameters=[{'config_path': fuzzy_config_path}]
        ),
        Node(
            package='state_machine',
            namespace='',
            executable='state_machine_node',
            name='state_machine',
        ),
        Node(
            package='march_gait_planning', 
            namespace='', 
            executable='gait_planning_node', 
            name='march_gait_planning', 
        ), 
        ipd_node, 
        mujoco_node,
        march_control,
        record_rosbags_action,
        safety_node,
        imu_nodes,
        ik_solver,
        state_estimator
    ])

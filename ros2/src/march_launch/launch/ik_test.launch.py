"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    mujoco_toload = LaunchConfiguration("model_to_load_mujoco", default='march8_v0.xml')
    tunings_to_load = LaunchConfiguration('tunings_to_load', default='low_level_controller_tunings.yaml')

    DeclareLaunchArgument(
        name="control_yaml",
        default_value="effort_control/march8_control.yaml",
        description="The controller yaml file to use loaded in through the controller manager "
                    "(not used if gazebo control is used). Must be in: `march_control/config/`.",
    )
    DeclareLaunchArgument(
        name="rviz", default_value="false", description="Whether we should startup rviz.", choices=["true", "false"]
    )
    DeclareLaunchArgument(
        name="simulation",
        default_value="false",
        description="Whether the exoskeleton is ran physically or in simulation.",
    )

    # region Launch Mujoco
    mujoco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("mujoco_sim"), "mujoco_sim.launch.py"])]
        ),
        launch_arguments=[("model_to_load", mujoco_toload), ("tunings_to_load_path", PathJoinSubstitution(
            [get_package_share_directory('march_control'), 'config', 'mujoco', tunings_to_load]))],
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
            ("layout", "training"),
        ],
    )

    ik_solver_launch_dir = os.path.join(
        get_package_share_directory('ik_solver'),
        'launch'
    )

    state_estimator_launch_dir = os.path.join(
        get_package_share_directory('state_estimator'),
        'launch'
    )

    urdf_location = os.path.join(
        get_package_share_directory('march_description'),
        'urdf',
        'hennie_with_koen.urdf'
    )

    # declare parameters
    # in ms
    trajectory_dt = 8

    return LaunchDescription([
        Node(
            package='bezier_visualization',
            executable='bezier_visualization_node',
            name='bezier_visualization',
        ),
        Node(
            package='footstep_generator',
            namespace='',
            executable='footstep_generator_node',
            name='footstep_generator'
        ),
        Node(
            package='swing_leg_trajectory_generator',
            namespace='',
            executable='swing_leg_trajectory_generator_node',
            name='swing_leg_generator'
        ),
        Node(
            package='ik_solver_buffer',
            namespace='',
            executable='ik_solver_buffer_node',
            name='ik_solver_buffer',
            parameters=[('timestep', str(trajectory_dt))],
        ),
        Node(
            package='state_machine',
            namespace='',
            executable='state_machine_node',
            name='state_machine',
        ),
        Node(
            package='gait_selection',
            namespace='',
            executable='gait_selection_node',
            name='gait_selection'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ik_solver_launch_dir, '/ik_solver_launch.py']),
            launch_arguments={'robot_description': urdf_location, "timestep": str(trajectory_dt)}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([state_estimator_launch_dir, '/state_estimator_launch.py']),
        ),
        mujoco_node,
        rqt_input_device,
        march_control,
    ])

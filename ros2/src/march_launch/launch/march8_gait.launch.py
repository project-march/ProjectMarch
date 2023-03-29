"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""

    mujoco_toload = LaunchConfiguration("model_to_load_mujoco", default='march.xml')
    tunings_to_load = LaunchConfiguration('tunings_to_load', default='low_level_controller_tunings.yaml')

    # region Launch Mujoco
    mujoco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("mujoco_sim"), "mujoco_sim.launch.py"])]
        ),
        launch_arguments=[("model_to_load", mujoco_toload), ("tunings_to_load_path", PathJoinSubstitution([get_package_share_directory('march_control'), 'config', 'mujoco', tunings_to_load]))],
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

    return LaunchDescription([
        Node(
            package='state_machine',
            namespace='',
            executable='state_machine_node',
            name='state_machine',
        ),
        Node(
            package='gait_loader',
            executable='gait_loader_node',
            name='gait_loader',
        ),
        mujoco_node,
        rqt_input_device,
    ])

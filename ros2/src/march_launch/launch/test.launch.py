"""Author: MARCH."""
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    """Generates the launch file for the march8 node structure."""
    mujoco_to_load = LaunchConfiguration("model_to_load_mujoco", default='march8_v0.xml')
    tunings_to_load = LaunchConfiguration('tunings_to_load', default='low_level_controller_tunings.yaml')
    simulation = LaunchConfiguration("simulation", default='true')
    robot = LaunchConfiguration("robot")

    DeclareLaunchArgument(
        name="robot",
        default_value="march8",
        description="The name of the yaml that will be used for retrieving info about the exo."
    )

    # region Launch Mujoco
    mujoco_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("mujoco_sim"), "mujoco_sim.launch.py"])]
        ),
        launch_arguments=[("model_to_load", mujoco_to_load), ("tunings_to_load_path", PathJoinSubstitution(
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

    urdf_location = os.path.join(
        get_package_share_directory('march_description'),
        'urdf',
        "march8",
        'hennie_with_koen.urdf'
    )

    return LaunchDescription([
        mujoco_node,
        march_control,

    ])

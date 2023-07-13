"""Created by: Rixt Hellinga, MVIII."""
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import Shutdown, DeclareLaunchArgument
import os


def generate_launch_description():
    """Generate the launch description for the fuzzy node."""    

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="left_ankle",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="right_ankle",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="left_knee",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="right_knee",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="left_hip_fe",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="right_hip_fe",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="left_hip_aa",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="right_hip_aa",
                default_value="-1.0",
                description="torque weight",
            ),
            DeclareLaunchArgument(
                name="absolute_max_torque",
                default_value="0.5",
                description="the absolute maximum torque percentage you can set directly in the recon tool",
            ),
            DeclareLaunchArgument(
                name="config_path",
                default_value= os.path.join(get_package_share_directory('fuzzy_generator'),'config','joints.yaml'),
                description="Path the the config file",
            ),
            DeclareLaunchArgument(
                name="allowed_control_type",
                default_value="position",
                description="Allowed control type",
            ),
            Node(
                package='fuzzy_generator',
                executable='fuzzy_node',
                name='fuzzy_node',
                parameters=[
                    {'config_path': LaunchConfiguration("config_path")},
                    {'allowed_control_type': LaunchConfiguration("allowed_control_type")},
                    {'left_ankle': LaunchConfiguration("left_ankle")},
                    {'right_ankle': LaunchConfiguration("right_ankle")},
                    {'left_knee': LaunchConfiguration("left_knee")},
                    {'right_knee': LaunchConfiguration("right_knee")},
                    {'left_hip_fe': LaunchConfiguration("left_hip_fe")},
                    {'right_hip_fe': LaunchConfiguration("right_hip_fe")},
                    {'left_hip_aa': LaunchConfiguration("left_hip_aa")},
                    {'right_hip_aa': LaunchConfiguration("right_hip_aa")},
                    {'absolute_max_torque': LaunchConfiguration("absolute_max_torque")},

            ]
    )
        ]

    )

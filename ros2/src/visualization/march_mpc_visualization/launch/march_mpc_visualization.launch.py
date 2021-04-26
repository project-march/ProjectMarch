import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> launch.LaunchDescription:
    return launch.LaunchDescription(
        [
           Node(
                package="march_mpc_visualization",
                executable="march_mpc_visualization",
                name="march_mpc_visualization",
                output="screen"
                #namespace="mpc_visualization",
                # output="screen",
                # arguments=[
                #     "--perspective-file",
                #     [
                #         PathJoinSubstitution(
                #             [
                #                 get_package_share_directory("march_monitor"),
                #                 "config",
                #                 LaunchConfiguration("perspective"),
                #             ]
                #         ),
                #         ".perspective",
                #     ],
                # ],
                # parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(
            #             get_package_share_directory("march_mpc_visualization"),
            #             "launch",
            #             "march_mpc_visualization.launch.py",
            #         )
            #     ),
            # ),
        ]
    )


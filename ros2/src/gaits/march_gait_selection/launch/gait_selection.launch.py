from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """ Basic launch file to launch the gait selection node """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            DeclareLaunchArgument(
                "gait_package",
                default_value="march_gait_files",
                description="The package where the gait files are located.",
            ),
            DeclareLaunchArgument(
                "gait_directory",
                default_value="training-v",
                description="The directory in which the gait files to use are "
                "located, relative to the gait_package.",
            ),
            DeclareLaunchArgument(
                "balance",
                default_value="False",
                description="Whether balance is being used.",
            ),
            Node(
                package="march_gait_selection",
                executable="march_gait_selection",
                output="screen",
                name="gait_selection",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"gait_package": LaunchConfiguration("gait_package")},
                    {"gait_directory": LaunchConfiguration("gait_directory")},
                    {"balance": LaunchConfiguration("balance")},
                ],
            ),
        ]
    )

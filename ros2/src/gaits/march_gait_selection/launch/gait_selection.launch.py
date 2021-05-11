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
            DeclareLaunchArgument(
                name="first_subgait_delay",
                default_value="0.2",
                description="Duration to wait before starting first subgait."
                "If 0 then the first subgait is started immediately,"
                "dropping the first setpoint in the process.",
            ),
            DeclareLaunchArgument(
                name="early_schedule_duration",
                default_value="0.2",
                description="Duration to schedule next subgait early. If 0 then the"
                "next subgait is never scheduled early.",
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
                    {"first_subgait_delay": LaunchConfiguration("first_subgait_delay")},
                    {
                        "early_schedule_duration": LaunchConfiguration(
                            "early_schedule_duration"
                        )
                    },
                ],
            ),
        ]
    )

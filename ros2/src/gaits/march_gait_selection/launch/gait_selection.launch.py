from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Basic launch file to launch the gait selection node"""
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
                default_value="airgait_vi",
                description="The directory in which the gait files to use are "
                "located, relative to the gait_package.",
            ),
            DeclareLaunchArgument(
                "balance",
                default_value="False",
                description="Whether balance is being used.",
            ),
            DeclareLaunchArgument(
                "dynamic_gait",
                default_value="False",
                description="Wether dynamic_setpoint_gait is enabled",
            ),
            # Dynamic gait parameters:
            DeclareLaunchArgument(
                name="dynamic_subgait_duration",
                default_value="1.5",
                description="Duration of a subgait created by the dynamic gait",
            ),
            DeclareLaunchArgument(
                name="middle_point_fraction",
                default_value="0.45",
                description="Fraction of the step at which the middle point "
                "of the dynamic gait will take place.",
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
            # State machine parameters:
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
            DeclareLaunchArgument(
                name="timer_period", default_value="0.004", description=""
            ),
            Node(
                package="march_gait_selection",
                executable="march_gait_selection",
                output="screen",
                name="gait_selection",
                namespace="march",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"gait_package": LaunchConfiguration("gait_package")},
                    {"gait_directory": LaunchConfiguration("gait_directory")},
                    {"balance": LaunchConfiguration("balance")},
                    {"dynamic_gait": LaunchConfiguration("dynamic_gait")},
                    {"dynamic_subgait_duration": 
                      LaunchConfiguration("dynamic_subgait_duration")
                    },
                    {"middle_point_fraction": 
                      LaunchConfiguration("middle_point_fraction")
                    },
                    {"middle_point_height": LaunchConfiguration("middle_point_height")},
                    {"minimum_stair_height": 
                      LaunchConfiguration("minimum_stair_height")
                    },
                    {"first_subgait_delay": LaunchConfiguration("first_subgait_delay")},
                    {
                        "early_schedule_duration": LaunchConfiguration(
                            "early_schedule_duration"
                        )
                    },
                    {"timer_period": LaunchConfiguration("timer_period")},
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

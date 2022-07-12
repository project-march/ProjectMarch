"""Author: Unknown."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Basic launch file to launch the gait selection node."""
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
                description="Whether dynamic_setpoint_gait is enabled",
            ),
            # Dynamic gait parameters:
            DeclareLaunchArgument(
                name="middle_point_fraction",
                default_value="0.45",
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
                default_value="True",
                description="Whether to add a push off setpoint for the ankle.",
            ),
            DeclareLaunchArgument(
                name="amount_of_steps",
                default_value="0",
                description="Amount of steps the dynamic gait should make before stopping. 0 or -1 is infinite.",
            ),
            DeclareLaunchArgument(
                name="use_position_queue",
                default_value="False",
                description="Uses the values in position_queue.yaml for the half step if True, otherwise uses "
                "points given by (simulated) covid.",
            ),
            DeclareLaunchArgument(
                name="add_cybathlon_gaits",
                default_value="False",
                description="Will add gaits created specifically for cybathlon obstacles to gait selection.",
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
                default_value="0.3",
                description="Duration to schedule next subgait early. If 0 then the"
                "next subgait is never scheduled early.",
            ),
            DeclareLaunchArgument(name="timer_period", default_value="0.004", description=""),
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
                    {"middle_point_fraction": LaunchConfiguration("middle_point_fraction")},
                    {"middle_point_height": LaunchConfiguration("middle_point_height")},
                    {"minimum_stair_height": LaunchConfiguration("minimum_stair_height")},
                    {"push_off_fraction": LaunchConfiguration("push_off_fraction")},
                    {"push_off_position": LaunchConfiguration("push_off_position")},
                    {"add_push_off": LaunchConfiguration("add_push_off")},
                    {"amount_of_steps": LaunchConfiguration("amount_of_steps")},
                    {"use_position_queue": LaunchConfiguration("use_position_queue")},
                    {"add_cybathlon_gaits": LaunchConfiguration("add_cybathlon_gaits")},
                    {"first_subgait_delay": LaunchConfiguration("first_subgait_delay")},
                    {"early_schedule_duration": LaunchConfiguration("early_schedule_duration")},
                    {"timer_period": LaunchConfiguration("timer_period")},
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

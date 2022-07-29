"""Author: Unknown."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from march_goniometric_ik_solver.ik_solver_parameters import IKSolverParameters


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
                default_value="0.5",
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
                default_value="false",
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
            DeclareLaunchArgument(
                name="fixed_midpoint_velocity",
                default_value="False",
                description="Will give all setpoints a velocity of zero if true.",
            ),
            DeclareLaunchArgument(
                name="stop_mid2_fraction",
                default_value="0.7",
                description="Fraction at which the second midpoint will be set for a stop gait."
            ),
            DeclareLaunchArgument(
                name="stop_mid2_x",
                default_value="0.03",
                description="X-location of the ankle for the second midpoint of a stop gait."
            ),
            DeclareLaunchArgument(
                name="stop_mid2_y",
                default_value="0.05",
                description="Y-location of the ankle for the second midpoint of a stop gait."
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
            DeclareLaunchArgument(name="timer_period", default_value="0.004", description=""),
            # IK solver parameters
            DeclareLaunchArgument(
                name="ankle_buffer",
                default_value=str(IKSolverParameters.ankle_buffer),
                description="buffer between dorsiflexion soft limit and allowed dorsiflexion in the ik solver, in deg",
            ),
            DeclareLaunchArgument(
                name="hip_buffer",
                default_value=str(IKSolverParameters.hip_buffer),
                description="buffer between retroflexion soft limit and allowed retroflexion in the ik solver, in deg",
            ),
            DeclareLaunchArgument(
                name="default_knee_bend",
                default_value=str(IKSolverParameters.default_knee_bend),
                description="efault knee flexion angle, in deg",
            ),
            DeclareLaunchArgument(
                name="hip_x_fraction",
                default_value=str(IKSolverParameters.hip_x_fraction),
                description="fraction of step at which hip is located",
            ),
            DeclareLaunchArgument(
                name="upper_body_front_rotation",
                default_value=str(IKSolverParameters.upper_body_front_rotation),
                description="forward tilt of the backpack, in deg",
            ),
            DeclareLaunchArgument(
                name="dorsiflexion_at_end_position",
                default_value=str(IKSolverParameters.dorsiflexion_at_end_position),
                description="Amount of dorsiflexion of swing leg ankle at end position. Takes regular ik solution "
                "if it is set to zero.",
            ),
            DeclareLaunchArgument(
                name="hip_swing",
                default_value=str(IKSolverParameters.hip_swing),
                description="Whether hip swing is enabled during walking.",
            ),
            DeclareLaunchArgument(
                name="base_number",
                default_value=str(IKSolverParameters.base_number),
                description="Base number of the function that calculates the ankle x mid position.",
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
                    {"middle_point_fraction": LaunchConfiguration("middle_point_fraction")},
                    {"middle_point_height": LaunchConfiguration("middle_point_height")},
                    {"minimum_stair_height": LaunchConfiguration("minimum_stair_height")},
                    {"push_off_fraction": LaunchConfiguration("push_off_fraction")},
                    {"push_off_position": LaunchConfiguration("push_off_position")},
                    {"add_push_off": LaunchConfiguration("add_push_off")},
                    {"amount_of_steps": LaunchConfiguration("amount_of_steps")},
                    {"use_position_queue": LaunchConfiguration("use_position_queue")},
                    {"add_cybathlon_gaits": LaunchConfiguration("add_cybathlon_gaits")},
                    {"fixed_midpoint_velocity": LaunchConfiguration("fixed_midpoint_velocity")},
                    {"stop_mid2_fraction": LaunchConfiguration("stop_mid2_fraction")},
                    {"stop_mid2_x": LaunchConfiguration("stop_mid2_x")},
                    {"stop_mid2_y": LaunchConfiguration("stop_mid2_y")},
                    {"base_number": LaunchConfiguration("base_number")},
                    {"first_subgait_delay": LaunchConfiguration("first_subgait_delay")},
                    {"early_schedule_duration": LaunchConfiguration("early_schedule_duration")},
                    {"timer_period": LaunchConfiguration("timer_period")},
                    {"ankle_buffer": LaunchConfiguration("ankle_buffer")},
                    {"hip_buffer": LaunchConfiguration("hip_buffer")},
                    {"default_knee_bend": LaunchConfiguration("default_knee_bend")},
                    {"hip_x_fraction": LaunchConfiguration("hip_x_fraction")},
                    {"upper_body_front_rotation": LaunchConfiguration("upper_body_front_rotation")},
                    {"dorsiflexion_at_end_position": LaunchConfiguration("dorsiflexion_at_end_position")},
                    {"hip_swing": LaunchConfiguration("hip_swing")},
                ],
                on_exit=Shutdown(),
            ),
        ]
    )

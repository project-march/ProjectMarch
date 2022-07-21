"""Author: MARCH."""
import os
import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
)

# Get lengths from urdf:
LENGTH_HIP_AA, LENGTH_HIP_BASE = get_lengths_robot_from_urdf_for_inverse_kinematics(
    length_names=["hip_aa_front", "hip_base"]
)
DEFAULT_FEET_DISTANCE = LENGTH_HIP_AA * 2 + LENGTH_HIP_BASE


def generate_launch_description() -> launch.LaunchDescription:
    """Generates the launch file for the simulation launch.

    This file extends the default march.launch file and overwrites the following default values:
        - [name] ([type]): from [old_val] -> to [new_val]
        - ...

    Todo:
        - Fill in the extended configuration values.
        - Fill in the settable ros parameters.

    The settable ros parameters are:
        use_sim_time (bool): Whether the node should use the simulation time as published on the /clock topic.
            Default is True.
        ...
    """
    # General arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot = LaunchConfiguration("robot")

    # Input device arguments
    rqt_input = LaunchConfiguration("rqt_input")
    ping_safety_node = LaunchConfiguration("ping_safety_node")
    layout = LaunchConfiguration("layout")

    # Robot state publisher arguments
    robot_state_publisher = LaunchConfiguration("robot_state_publisher")
    robot_description = LaunchConfiguration("robot_description")
    ground_gait = LaunchConfiguration("ground_gait")
    use_imu_data = LaunchConfiguration("use_imu_data")
    imu_to_use = LaunchConfiguration("imu_to_use")
    imu_topic = LaunchConfiguration("imu_topic")
    simulation = LaunchConfiguration("simulation")
    jointless = LaunchConfiguration("jointless")

    # Simulation arguments
    realsense_simulation = LaunchConfiguration("realsense_simulation")
    to_world_transform = LaunchConfiguration("to_world_transform")

    # Gait selection arguments
    gait_package = LaunchConfiguration("gait_package")
    gait_directory = LaunchConfiguration("gait_directory")
    balance = LaunchConfiguration("balance")
    dynamic_gait = LaunchConfiguration("dynamic_gait")
    middle_point_fraction = LaunchConfiguration("middle_point_fraction")
    middle_point_height = LaunchConfiguration("middle_point_height")
    minimum_stair_height = LaunchConfiguration("minimum_stair_height")
    push_off_fraction = LaunchConfiguration("push_off_fraction")
    push_off_position = LaunchConfiguration("push_off_position")
    add_push_off = LaunchConfiguration("add_push_off")
    amount_of_steps = LaunchConfiguration("amount_of_steps")
    use_position_queue = LaunchConfiguration("use_position_queue")
    add_cybathlon_gaits = LaunchConfiguration("add_cybathlon_gaits")
    first_subgait_delay = LaunchConfiguration("first_subgait_delay")
    early_schedule_duration = LaunchConfiguration("early_schedule_duration")
    timer_period = LaunchConfiguration("timer_period")

    # Fake sensor data
    fake_sensor_data = LaunchConfiguration("fake_sensor_data")
    minimum_fake_temperature = LaunchConfiguration("minimum_fake_temperature")
    maximum_fake_temperature = LaunchConfiguration("maximum_fake_temperature")

    # Gait Preprocessor
    location_x = LaunchConfiguration("location_x")
    location_y = LaunchConfiguration("location_y")
    duration = LaunchConfiguration("duration")
    location_z = LaunchConfiguration("location_z")

    return launch.LaunchDescription(
        [
            # GENERAL ARGUMENTS
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            DeclareLaunchArgument(name="robot", default_value="march7", description="Robot to use."),
            # RQT INPUT DEVICE ARGUMENTS
            DeclareLaunchArgument(
                name="rqt_input",
                default_value="True",
                description="If this argument is false, the rqt input device will not be launched.",
            ),
            DeclareLaunchArgument(
                name="layout",
                default_value="training",
                description="Input device layout .json file to use.",
            ),
            DeclareLaunchArgument(
                name="ping_safety_node",
                default_value="True",
                description="Whether the input device should ping the safety node"
                "with an alive message every 0.2 seconds",
            ),
            # ROBOT STATE PUBLISHER ARGUMENTS
            DeclareLaunchArgument(
                name="robot_state_publisher",
                default_value="True",
                description="Whether or not to launch the robot state publisher,"
                "this allows nodes to get the urdf and to subscribe to"
                "potential urdf updates. This is necesary for gait selection"
                "to be able to launch",
            ),
            DeclareLaunchArgument(
                name="robot_description",
                default_value=robot,
                description="Which <robot_description>.xacro file to use. "
                "This file must be available in the march_desrciption/urdf/ folder",
            ),
            DeclareLaunchArgument(
                name="realsense_simulation",
                default_value="false",
                description="Whether the simulation camera or the physical camera should be used",
            ),
            DeclareLaunchArgument(
                name="use_imu_data",
                default_value="False",
                description="Whether to use the camera imu to know the real orientation of the exoskeleton",
            ),
            DeclareLaunchArgument(
                name="ground_gait",
                default_value=use_imu_data,
                description="Whether the simulation should be simulating ground_gaiting instead of airgaiting.",
            ),
            DeclareLaunchArgument(
                name="to_world_transform",
                default_value=ground_gait,
                description="Whether a transform from the world to base_link is "
                "necessary, this is the case when you are "
                "groundgaiting in rviz.",
            ),
            DeclareLaunchArgument(name="imu_to_use", default_value="back", description="Which imu to use"),
            DeclareLaunchArgument(
                name="imu_topic",
                default_value="/camera_back/imu/data",
                description="The topic that should be used to determine the orientation",
            ),
            DeclareLaunchArgument(
                name="simulation",
                default_value="True",
                description="Whether simulation is used",
            ),
            # GAIT SELECTION ARGUMENTS
            DeclareLaunchArgument(
                name="gait_package",
                default_value="march_gait_files",
                description="The package where the gait files are located.",
            ),
            DeclareLaunchArgument(
                name="gait_directory",
                default_value="airgait_vi",
                description="The directory in which the gait files to use are located, "
                "relative to the gait_package.",
            ),
            DeclareLaunchArgument(
                name="balance",
                default_value="False",
                description="Whether balance is being used.",
            ),
            DeclareLaunchArgument(
                name="dynamic_gait",
                default_value="True",
                description="Whether dynamic_setpoint_gait is enabled",
            ),
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
            DeclareLaunchArgument(
                "timer_period",
                default_value="0.004",
                description="",
            ),
            DeclareLaunchArgument(
                "jointless",
                default_value="False",
                description="If true, no joints will be actuated",
            ),
            # FAKE SENSOR DATA ARGUMENTS
            DeclareLaunchArgument(
                name="fake_sensor_data",
                default_value="False",
                description="Whether to launch the fake sensor data node.",
            ),
            DeclareLaunchArgument(
                "minimum_fake_temperature",
                default_value="10",
                description="Lower bound to generate fake temperatures from",
            ),
            DeclareLaunchArgument(
                "maximum_fake_temperature",
                default_value="30",
                description="Upper bound to generate fake temperatures from",
            ),
            # GAIT PREPROCESSOR ARGUMENTS
            DeclareLaunchArgument(
                name="location_x",
                default_value="0.5",
                description="x-location for fake covid topic, takes double or 'random'",
            ),
            DeclareLaunchArgument(
                name="location_y",
                default_value="0.03",
                description="y-location for fake covid topic, takes double or 'random'",
            ),
            DeclareLaunchArgument(
                name="location_z",
                default_value=str(DEFAULT_FEET_DISTANCE),
                description="z-location for fake covid topic, takes double or 'random'",
            ),
            DeclareLaunchArgument(
                name="duration",
                default_value="1.5",
                description="Base duration of dynamic gait, may be scaled depending on step height",
            ),
            # Use normal launch file with different launch_arguments
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("march_launch"),
                        "launch",
                        "march.launch.py",
                    )
                ),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("rqt_input", rqt_input),
                    ("ping_safety_node", ping_safety_node),
                    ("layout", layout),
                    ("robot", robot),
                    ("robot_state_publisher", robot_state_publisher),
                    ("use_imu_data", use_imu_data),
                    ("imu_topic", imu_topic),
                    ("imu_to_use", imu_to_use),
                    ("robot_description", robot_description),
                    ("ground_gait", ground_gait),
                    ("realsense_simulation", realsense_simulation),
                    ("to_world_transform", to_world_transform),
                    ("gait_package", gait_package),
                    ("gait_directory", gait_directory),
                    ("balance", balance),
                    ("dynamic_gait", dynamic_gait),
                    ("duration", duration),
                    ("middle_point_fraction", middle_point_fraction),
                    ("middle_point_height", middle_point_height),
                    ("minimum_stair_height", minimum_stair_height),
                    ("push_off_fraction", push_off_fraction),
                    ("push_off_position", push_off_position),
                    ("add_push_off", add_push_off),
                    ("amount_of_steps", amount_of_steps),
                    ("use_position_queue", use_position_queue),
                    ("add_cybathlon_gaits", add_cybathlon_gaits),
                    ("first_subgait_delay", first_subgait_delay),
                    ("early_schedule_duration", early_schedule_duration),
                    ("timer_period", timer_period),
                    ("fake_sensor_data", fake_sensor_data),
                    ("minimum_fake_temperature", minimum_fake_temperature),
                    ("maximum_fake_temperature", maximum_fake_temperature),
                    ("simulation", simulation),
                    ("jointless", jointless),
                    ("location_x", location_x),
                    ("location_y", location_y),
                    ("location_z", location_z),
                ],
            ),
        ]
    )

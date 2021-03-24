import os
import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
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
    imu_topic = LaunchConfiguration("imu_topic")

    # Simulation arguments
    realsense = LaunchConfiguration("realsense")
    realsense_simulation = LaunchConfiguration("realsense_simulation")
    to_world_transform = LaunchConfiguration("to_world_transform")

    # Gait selection arguments
    gait_package = LaunchConfiguration("gait_package")
    gait_directory = LaunchConfiguration("gait_directory")
    balance = LaunchConfiguration("balance")

    # Fake sensor data
    fake_sensor_data = LaunchConfiguration("fake_sensor_data")
    minimum_fake_temperature = LaunchConfiguration("minimum_fake_temperature")
    maximum_fake_temperature = LaunchConfiguration("maximum_fake_temperature")

    return launch.LaunchDescription(
        [
            # GENERAL ARGUMENTS
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            DeclareLaunchArgument(
                name="robot",
                default_value="march4",
                description="Robot to use."
            ),
            # RQT INPUT DEVICE ARGUMENTS
            DeclareLaunchArgument(
                name="rqt_input",
                default_value="True",
                description="If this argument is false, the rqt input device will"
                "not be launched.",
            ),
            DeclareLaunchArgument(
                name="layout",
                default_value="default",
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
                default_value="True",
                description="Whether the simulation camera or the physical camera should be used",
            ),
            DeclareLaunchArgument(
                name="ground_gait",
                default_value="False",
                description="Whether the simulation should be simulating "
                "ground_gaiting instead of airgaiting.",
            ),
            DeclareLaunchArgument(
                name="to_world_transform",
                default_value=ground_gait,
                description="Whether a transform from the world to base_link is "
                "necessary, this is the case when you are "
                "groundgaiting in rviz.",
            ),
            DeclareLaunchArgument(
                name="use_imu_data",
                default_value="False",
                description="Whether to use the camera imu to know the real "
                            "orientation of the exoskeleton"
            ),
            DeclareLaunchArgument(
                name="imu_topic",
                default_value="/camera_front/imu/data",
                description="The topic that should be used to determine the orientation"
            ),
            # GAIT SELECTION ARGUMENTS
            DeclareLaunchArgument(
                name="gait_package",
                default_value="march_gait_files",
                description="The package where the gait files are located.",
            ),
            DeclareLaunchArgument(
                name="gait_directory",
                default_value="test_versions-vi",
                description="The directory in which the gait files to use are located, "
                "relative to the gait_package.",
            ),
            DeclareLaunchArgument(
                name="balance",
                default_value="False",
                description="Whether balance is being used.",
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
                    ("robot_description", robot_description),
                    ("ground_gait", ground_gait),
                    ("realsense_simulation", realsense_simulation),
                    ("to_world_transform", to_world_transform),
                    ("gait_package", gait_package),
                    ("gait_directory", gait_directory),
                    ("balance", balance),
                    ("fake_sensor_data", fake_sensor_data),
                    ("minimum_fake_temperature", minimum_fake_temperature),
                    ("maximum_fake_temperature", maximum_fake_temperature),
                ],
            ),
        ]
    )

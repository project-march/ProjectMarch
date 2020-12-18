import os
import launch
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # General arguments
    node_prefix = LaunchConfiguration('node_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot = LaunchConfiguration('robot')
    # Input device arguments
    rqt_input = LaunchConfiguration('rqt_input')
    ping_safety_node = LaunchConfiguration('ping_safety_node')
    # Robot state publisher arguments
    robot_state_publisher = LaunchConfiguration('robot_state_publisher')
    robot_description = LaunchConfiguration('robot_description')
    # Gait selection arguments
    gait_selection = LaunchConfiguration('gait_selection')
    gait_package = LaunchConfiguration('gait_package')
    gait_directory = LaunchConfiguration('gait_directory')
    # Fake sensor data
    fake_sensor_data = LaunchConfiguration('fake_sensor_data')
    minimum_fake_temperature = LaunchConfiguration('minimum_fake_temperature')
    maximum_fake_temperature = LaunchConfiguration('maximum_fake_temperature')

    return launch.LaunchDescription([
        # GENERAL ARGUMENTS
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Whether to use simulation time as published on the '
                        '/clock topic by gazebo instead of system time.'),
        DeclareLaunchArgument(
            name='robot',
            default_value='march4',
            description="Robot to use."
        ),
        # RQT INPUT DEVICE ARGUMENTS
        DeclareLaunchArgument(
            name='rqt_input',
            default_value='True',
            description='If this argument is false, the rqt input device will'
                        'not be launched.'),
        DeclareLaunchArgument(
            name='ping_safety_node',
            default_value='True',
            description='Whether the input device should ping the safety node'
                        'with an alive message every 0.2 seconds'),
        # ROBOT STATE PUBLISHER ARGUMENTS
        DeclareLaunchArgument(
            name='robot_state_publisher',
            default_value='True',
            description='Whether or not to launch the robot state publisher,'
                        'this allows nodes to get the urdf and to subscribe to'
                        'potential urdf updates. This is necesary for gait selection'
                        'to be able to launch'),
        DeclareLaunchArgument(
            name='robot_description',
            default_value=robot,
            description="Which <robot_description>.xacro file to use. "
                        "This file must be available in the march_desrciption/urdf/ folder"
        ),
        # GAIT SELECTION ARGUMENTS
        DeclareLaunchArgument(
            name='gait_selection',
            default_value='True',
            description='Whether to launch the march gait selection node.'),
        DeclareLaunchArgument(
            'gait_package',
            default_value='march_gait_files',
            description='The package where the gait files are located.'),
        DeclareLaunchArgument(
            'gait_directory',
            default_value='training-v',
            description='The directory in which the gait files to use are located, '
                        'relative to the gait_package.'),
        # Launch rqt input device if not rqt_input:=false
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_rqt_input_device'), 'launch', 'input_device.launch.py')),
            launch_arguments=[('ping_safety_node', ping_safety_node),
                              ('use_sim_time', use_sim_time)],
            condition=IfCondition(rqt_input)),
        # Launch robot state publisher (from march_description) if not robot_state_publisher:=false
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_description'), 'launch', 'march_description.launch.py')),
            launch_arguments=[('robot_description', robot_description),
                              ('use_sim_time', use_sim_time)],
            condition=IfCondition(robot_state_publisher)),
        # Launch march gait selection if not gait_selection:=false
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_gait_selection'), 'launch', 'gait_selection.launch.py')),
            launch_arguments=[('gait_directory', gait_directory),
                              ('use_sim_time', use_sim_time),
                              ('gait_package', gait_package)],
            condition=IfCondition(gait_selection)),

        # Fake sensor data
        DeclareLaunchArgument(
            name='fake_sensor_data',
            default_value='False',
            description='Whether to launch the fake sensor data node.'),
        DeclareLaunchArgument(
            'minimum_fake_temperature',
            default_value='10',
            description='Lower bound to generate fake temperatures from'),
        DeclareLaunchArgument(
            'maximum_fake_temperature',
            default_value='30',
            description='Upper bound to generate fake temperatures from'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('march_fake_sensor_data'), 'launch', 'march_fake_sensor_data.launch.py')),
            launch_arguments=[('minimum_fake_temperature', minimum_fake_temperature),
                              ('maximum_fake_temperature', maximum_fake_temperature)],
            condition=IfCondition(fake_sensor_data))
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# <arg name="moticon_ip" default="192.168.8.105" doc="The ip-adress with Moticon software running on it, defaults to
# EMS switch laptop on standard router"/>
# <arg name="pressure_soles" default="false" doc="Whether pressure soles will be connected"/>
# <arg name="logfile" default="false" doc="Whether the data input is from a log file" />
# <arg name="robot" default="march4" doc="The robot to run. Can be: march3, march4, test_joint_linear, test_joint_rotational."/>


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
                "moticon_ip",
                default_value="192.168.8.105",
                description="The ip-adress with Moticon software running on it, "
                "defaults to EMS switch laptop on standard router",
            ),
            DeclareLaunchArgument(
                "pressure_soles",
                default_value="false",
                description="Whether the pressure soles will be connected",
            ),
            DeclareLaunchArgument(
                "logfile",
                default_value="false",
                description="Whether the data input is from a log file.",
            ),
            Node(
                package="march_data_collector",
                executable="data_collector",
                output="screen",
                name="data_collector",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"moticon_ip": LaunchConfiguration("moticon_ip")},
                    {"pressure_soles": LaunchConfiguration("pressure_soles")},
                    {"logfile": LaunchConfiguration("logfile")},
                ],
            ),
        ]
    )

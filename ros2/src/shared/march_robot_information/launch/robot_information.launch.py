from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """ Basic launch file to launch the robot information node """
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Whether to use simulation time as published on the "
                "/clock topic by gazebo instead of system time.",
            ),
            DeclareLaunchArgument(
                name="joint_names",
                default_value="None",
                description="Names of the joints. If an empty list (None) is given"
                "as argument the robot_information node uses the "
                "robot_description parameter of the "
                "robot_state_publisher to determine the joint names.",
            ),
            Node(
                package="march_robot_information",
                executable="march_robot_information",
                output="screen",
                name="robot_information",
                namespace="march",
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"joint_names": LaunchConfiguration("joint_names")},
                ],
            ),
        ]
    )

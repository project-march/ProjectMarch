import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_CONTROLLER_FILE = os.path.join("gazebo", "march6_control.yaml")


def generate_launch_description():
    controller_file = LaunchConfiguration("controller_file")
    robot_description_file = LaunchConfiguration("robot_description")
    ground_gait = LaunchConfiguration("ground_gait")

    declared_arguments = [
        DeclareLaunchArgument(
            "controller_file",
            default_value=DEFAULT_CONTROLLER_FILE,
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "robot_description",
            default_value="march6_ros2",
            description="XACRO or URDF file containing the robot model.",
        ),
        DeclareLaunchArgument(
            "ground_gait",
            default_value="false",
            description="Whether we want to have exo be stuck in the air, or walking over the ground.",
        )
    ]

    # region Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    # Will create an entity with the name -entity [name] based on the urdf in the topic -topic [topic]
    # Check this link for more information: https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Spawn-and-delete
    gazebo_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "fred"],
        output="screen",
    )
    # endregion

    # robot_description_content = open("/home/george/repos/ros2_demo_test/ros2_control_demos/src/ros2_control_demo_description/rrbot_description/urdf/rrbot.urdf.xacro").read()

    #region Launch Robot state publisher. Publishes the robot description on '/robot_description'
    robot_state_pub_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_description"),
                "launch",
                "march_description.launch.py",
            )
        ),
        launch_arguments=[
            ("robot_description", robot_description_file),
            ("use_sim_time", 'true'),
            ("realsense_simulation", 'true'),
            ("ground_gait", ground_gait),
            ("simulation", 'true'),
            ("jointless", 'false'),
            ("control_yaml", controller_file)
        ],
    )
    # endregion

    # region Launch RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("march_launch"), "rviz", "ros2.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )
    rviz_node_delay_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_entity,
            on_exit=[rviz_node],
        )
    )
    # endregion

    # region Launch march control
    march_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_control"),
                "launch",
                "controllers.launch.py",
            )
        ),
        launch_arguments=[
            ("simulation", 'true'),
        ],
    )
    # endregion

    nodes = [
        robot_state_pub_node,
        gazebo,
        gazebo_spawn_entity,
        march_control,
        rviz_node_delay_after_gazebo,  # Rviz after broadcast spawner.
    ]

    return LaunchDescription(declared_arguments + nodes)

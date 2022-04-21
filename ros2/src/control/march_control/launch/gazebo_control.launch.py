# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_CONTROLLER_FILE = os.path.join("gazebo", "march6_control.yaml")


def generate_launch_description():
    controller_file = LaunchConfiguration("controller_file")
    robot_description_file = LaunchConfiguration("robot_description")
    ground_gait = LaunchConfiguration("ground_gait")

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_file",
            default_value=DEFAULT_CONTROLLER_FILE,
            description="YAML file with the controllers configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description",
            default_value="march6_ros2",
            description="XACRO or URDF file containing the robot model.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ground_gait",
            default_value="false",
            description="Whether we want to have exo be stuck in the air, or walking over the ground.",
        )
    )

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

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("march_description"),
                    "urdf",
                    robot_description_file
                ]
            ),
            ".xacro ",
            "groundgait:=",
            ground_gait,
            " gazebo_control:=true configuration:=simulation",
        ]
    )

    # robot_description_content = open("/home/george/repos/ros2_demo_test/ros2_control_demos/src/ros2_control_demo_description/rrbot_description/urdf/rrbot.urdf.xacro").read()

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("march_control"),
            "config",
            controller_file,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],  # Here the robot desc is loaded in and the controller yaml.
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    nodes = [
        gazebo,
        gazebo_spawn_entity,
        rviz_node,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner
    ]

    return LaunchDescription(declared_arguments + nodes)

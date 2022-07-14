from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from march_utility.utilities.build_tool_functions import get_control_file_loc


def generate_launch_description():
    # Whether the exoskeleton is ran physically or in simulation.
    simulation = LaunchConfiguration("simulation")
    rviz = LaunchConfiguration("rviz")
    gazebo = LaunchConfiguration("gazebo")
    control_type = LaunchConfiguration("control_type")
    actuating = LaunchConfiguration("actuating")

    declared_arguments = [
        DeclareLaunchArgument(
            name="simulation",
            default_value="true",
            description="Whether the exoskeleton is ran physically or in simulation.",
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            name="rviz",
            default_value="true",
            description="Whether we should startup rviz.",
            choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            name="actuating",
            default_value="true",
            description="Whether we should only read the values of the joints (thus not send actuation commands).",
            choices=["true", "false"]
        ),
        # region set control type. (This section set the control types for either gazebo, exo, or rviz control)
        DeclareLaunchArgument(
            name="control_type",
            default_value="effort",
            description="Decides which controller is being used. "
                        "'Effort' when you are running either gazebo or the real exo",
            choices=["rviz", "effort"],
            condition=UnlessCondition(PythonExpression(["'", simulation, "' == 'true'", " and '", gazebo, "' == 'false'"])),
        ),
        DeclareLaunchArgument(
            name="control_type",
            default_value="rviz",
            description="Decides which controller is being used. "
                        "'Rviz' when you are not running either gazebo or the real exo",
            choices=["rviz", "effort"],
            # condition=[launch.conditions.IfCondition(simulation), launch.conditions.UnlessCondition(gazebo)],
            condition=IfCondition(PythonExpression(["'", simulation, "' == 'true'", " and '", gazebo, "' == 'false'"])),
        )
        # endregion
    ]


    # region Starts Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(actuating),
    )

    # endregion

    nodes = [
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ]

    # region Launch Controller manager, Extra configuration if simulation is `false`
    # This is section is not needed in simulation as control is taken over by gazebo control.
    # In gazebo the controller is loaded in by the xacro file.
    control_yaml = LaunchConfiguration("control_yaml")

    robot_desc_xacro = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("march_control"), "xacro", "ros2_control.xacro"]),
            " type:=",
            control_type,
            " gazebo:=",
            gazebo,
        ]
    )
    robot_desc_dict = {"robot_description": robot_desc_xacro}

    # This node couples the HW interface with control.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # The robot description is loaded in and the controller yaml.
        parameters=[robot_desc_dict, get_control_file_loc(control_yaml)],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=UnlessCondition(gazebo),
    )
    nodes.append(control_node)
    # endregion

    # region Launch RViz
    # This is defined here due to that otherwise it spams the terminal with not found tf frames,
    # if not started after the robot description publisher.
    rviz_config_file = PathJoinSubstitution([FindPackageShare("march_launch"), "rviz", "ros2.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )
    rviz_node_after_broadcast_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    nodes.append(rviz_node_after_broadcast_spawner)
    # endregion

    return LaunchDescription(declared_arguments + nodes)

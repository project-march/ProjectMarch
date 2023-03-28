"""Author: George Vegelien, MVII."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition, IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    PythonExpression,
    EnvironmentVariable,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from march_utility.utilities.build_tool_functions import get_control_file_loc


def generate_launch_description():
    """Launch file to start up the controllers, controller_manager, hardware_interface and rviz."""
    # Whether the exoskeleton is ran physically or in simulation.
    simulation = LaunchConfiguration("simulation")
    rviz = LaunchConfiguration("rviz")
    gazebo = LaunchConfiguration("gazebo")
    control_type = LaunchConfiguration("control_type")
    actuating = LaunchConfiguration("actuating")

    rviz_condition = PythonExpression(["'", simulation, "' == 'true'", " and '", gazebo, "' == 'false'"])

    declared_arguments = [
        DeclareLaunchArgument(
            name="simulation",
            default_value="true",
            description="Whether the exoskeleton is ran physically or in simulation.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            name="rviz", default_value="true", description="Whether we should startup rviz.", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            name="actuating",
            default_value="true",
            description="Whether we should only read the values of the joints (thus not send actuation commands).",
            choices=["true", "false"],
        ),
        # region set control type. (This section set the control types for either gazebo, exo, or rviz control)
        DeclareLaunchArgument(
            name="control_type",
            default_value="effort",
            description="Decides which controller is being used. "
            "'Effort' when you are running either gazebo or the real exo",
            choices=["rviz", "effort"],
            condition=UnlessCondition(rviz_condition),
        ),
        DeclareLaunchArgument(
            name="control_type",
            default_value="rviz",
            description="Decides which controller is being used. "
            "'Rviz' when you are not running either gazebo or the real exo",
            choices=["rviz", "effort"],
            condition=IfCondition(rviz_condition),
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

    # region Start broadcasters
    pdb_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "pdb_state_broadcaster",
            "-t",
            "pdb_state_broadcaster/PdbStateBroadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(simulation),
    )

    motor_controller_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "march_motor_controller_state_broadcaster",
            "-t",
            "march_motor_controller_state_broadcaster/MotorControllerStateBroadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(simulation),
    )

    # region Start broadcasters
    pressure_sole_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=[
            "pressure_sole_broadcaster",
            "-t",
            "pressure_sole_broadcaster/PressureSoleBroadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(simulation),
    )
    # endregion

    nodes = [
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        pdb_state_broadcaster_spawner,
        motor_controller_state_broadcaster_spawner,
        pressure_sole_state_broadcaster_spawner,
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
    control_node_exo = Node(
        package="controller_manager",
        prefix=[  # Sudo command cause need to be sudoer when we do this node cause it real time
            "sudo -s -E env PATH=",
            EnvironmentVariable("PATH", default_value="${PATH}"),
            " LD_LIBRARY_PATH=",
            EnvironmentVariable("LD_LIBRARY_PATH", default_value="${LD_LIBRARY_PATH}"),
            " PYTHONPATH=",
            EnvironmentVariable("PYTHONPATH", default_value="${PYTHONPATH}"),
            " HOME=/tmp ",
        ],
        executable="ros2_control_node",
        # The robot description is loaded in and the controller yaml.
        parameters=[robot_desc_dict, get_control_file_loc(control_yaml)],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=IfCondition(PythonExpression(["'", control_type, "' == 'effort'", " and '", gazebo, "' == 'false'"])),
    )
    # This should be the same control node as the one above only that the prefix for sudo is not needed in simulation.
    control_node_rviz = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # The robot description is loaded in and the controller yaml.
        parameters=[robot_desc_dict, get_control_file_loc(control_yaml)],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=IfCondition(PythonExpression(["'", control_type, "' == 'rviz'", " and '", gazebo, "' == 'false'"])),
    )
    nodes.extend([control_node_rviz, control_node_exo])  # noqa: PIE799 This way is more readable.
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
        condition=IfCondition(rviz),
    )
    rviz_node_after_broadcast_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    nodes.append(rviz_node_after_broadcast_spawner)  # noqa: PIE799 This way is more readable and easier to uncomment.
    # endregion

    return LaunchDescription(declared_arguments + nodes)

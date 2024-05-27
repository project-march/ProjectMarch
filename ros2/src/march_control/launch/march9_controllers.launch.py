"""Author: Marco Bak, MVIII."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
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
    """Launch file to start up the controllers, controller_manager, hardware_interface and simulation."""
    # Whether the exoskeleton is ran physically or in simulation.
    simulation = LaunchConfiguration("simulation")
    control_type = LaunchConfiguration("control_type")
    # region Launch Controller manager, Extra configuration if simulation is `false`

    control_yaml = LaunchConfiguration("control_yaml")

    declared_arguments = [
        DeclareLaunchArgument(
            name="simulation",
            default_value="true",
            description="Whether the exoskeleton is ran physically or in simulation.",
            choices=["true", "false"],
        ),
        # region set control type. (This section set the control types for either gazebo, exo, or simulation control)
        DeclareLaunchArgument(
            name="control_type",
            default_value="effort",
            description="Decides which controller is being used. " "'Effort' when you are running the real exo",
            choices=["simulation", "effort"],
            condition=UnlessCondition(simulation),
        ),
        DeclareLaunchArgument(
            name="control_type",
            default_value="simulation",
            description="Decides which controller is being used. " "'simulation' when you are running the simulation",
            choices=["simulation", "effort"],
            condition=IfCondition(simulation),
        ),
        DeclareLaunchArgument(
            name="control_yaml",
            default_value="effort_control/march9_control.yaml",
            description="The controller yaml file to use loaded in through the controller manager "
            "(not used if gazebo control is used). Must be in: `march_control/config/`.",
            condition=UnlessCondition(simulation),
        ),
        DeclareLaunchArgument(
            name="control_yaml",
            default_value="mujoco/march9_control.yaml",
            description="The controller yaml file to use loaded in through the controller manager "
            "(not used if gazebo control is used). Must be in: `march_control/config/`.",
            condition=IfCondition(simulation),
        ),
        # endregion
    ]

    # region Starts Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["march_joint_position_controller", "--controller-manager", "/controller_manager"],
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
    # endregion

    nodes = [
        joint_state_broadcaster_spawner,
        pdb_state_broadcaster_spawner,
        motor_controller_state_broadcaster_spawner,
        joint_position_controller_spawner,
    ]

    robot_desc_xacro = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("march_control"), "xacro", "ros2_control_m9.xacro"]),
            " type:=",
            control_type,
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
        condition=IfCondition(PythonExpression(["'", control_type, "' == 'effort'"])),
    )
    # This should be the same control node as the one above only that the prefix for sudo is not needed in simulation.
    control_node_sim = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # The robot description is loaded in and the controller yaml.
        parameters=[robot_desc_dict, get_control_file_loc(control_yaml)],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        condition=IfCondition(PythonExpression(["'", control_type, "' == 'simulation'"])),
    )
    nodes.extend([control_node_sim, control_node_exo])  # noqa: PIE799 This way is more readable.
    # endregion

    return LaunchDescription(declared_arguments + nodes)

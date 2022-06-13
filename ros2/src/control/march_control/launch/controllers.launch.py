from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from march_utility.utilities.build_tool_functions import generate_robot_desc_command, get_control_file_loc


def generate_launch_description():
    # Whether the exoskeleton is ran physically or in simulation.
    simulation = LaunchConfiguration("simulation")

    declared_arguments = [
        DeclareLaunchArgument(
            name="simulation",
            default_value="false",
            description="Whether the exoskeleton is ran physically or in simulation.",
        )
    ]

    # region Controllers
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

    # endregion

    nodes = [
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ]

    # region Extra configuration if simulation is `false`
    # This is section is not needed in simulation as control is taken over by gazebo control.
    # In gazebo the controller is loaded in by the xacro file.

    # region Generate Robot desc (or known as xacro/urdf)
    """Because the following are needed by xacro and should be the same as the ones given to control,
    they do not have a DeclaredLaunchArgument to ensure these are defined at a higher level that this launch file.
    """
    # Which <robot_description>.xacro file to use. This file must be available in the `march_description/urdf/` folder.
    robot_description = LaunchConfiguration("robot_description")
    # Whether the simulation should be simulating ground_gaiting instead of airgaiting.
    ground_gait = LaunchConfiguration("ground_gait")
    # Whether the simulation camera or the physical camera should be used.
    realsense_simulation = LaunchConfiguration("realsense_simulation")
    # If true, no joints will be actuated.
    jointless = LaunchConfiguration("jointless")
    # Path to the control file. Must be in `march_control/config/gazebo/`.
    control_yaml = LaunchConfiguration("control_yaml")

    robot_desc_xacro = Command(
        generate_robot_desc_command(
            robot_descr_file=robot_description,
            ground_gait=ground_gait,
            realsense_simulation=realsense_simulation,
            simulation=simulation,
            jointless=jointless,
            control_yaml=control_yaml,
        )
    )
    robot_desc_dict = {"robot_description": robot_desc_xacro}
    # endregion
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
        condition=UnlessCondition(simulation),
    )
    nodes.append(control_node)
    # endregion

    return LaunchDescription(declared_arguments + nodes)

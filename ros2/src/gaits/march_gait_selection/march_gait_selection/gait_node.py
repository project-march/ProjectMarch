"""Author: Marten Haitjema, MVII."""

import signal
import sys
import rclpy

from rcl_interfaces.msg import SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from contextlib import suppress

from march_gait_selection.gait_loader import GaitLoader
from march_gait_selection.state_machine.gait_state_machine import GaitStateMachine

from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler
from march_utility.utilities.duration import Duration
from march_utility.utilities.node_utils import get_robot_urdf_from_service

NODE_NAME = "gait"


def sys_exit(*_):
    """Cleanly exit."""
    sys.exit(0)


def main():
    """Starts up the gait selection node with the state machine and scheduler."""
    rclpy.init()

    node = GaitNode()
    robot = get_robot_urdf_from_service(node)

    gait_loader = GaitLoader(node, robot)
    scheduler = TrajectoryScheduler(node)
    gait_state_machine = GaitStateMachine(node, scheduler, gait_loader.gaits, gait_loader.positions)
    gait_state_machine.run()

    node.add_on_set_parameters_callback(lambda params: parameter_callback(node, gait_state_machine, params))

    signal.signal(signal.SIGTERM, sys_exit)

    executor = MultiThreadedExecutor()
    with suppress(KeyboardInterrupt):
        rclpy.spin(node, executor)

    rclpy.shutdown()


def parameter_callback(node: Node, gait_state_machine: GaitStateMachine, parameters) -> SetParametersResult:
    """A callback function that is used to update the parameters that are a part of the gait selection node.

    Since some of these parameters are from the gait_state_machine, some from gait_selection and some from both, this is
    implemented here.

    Args:
        node (Node): The node that is used in the GaitLoader and GaitStateMachine
        gait_state_machine (GaitStateMachine): The GaitStateMachine object, to access the gait objects
        parameters (???): The parameters to update
    Returns:
        SetParametersResult: Whether the callback was successful
    """
    position_queue_updated = False
    dynamic_gait_updated = False
    for param in parameters:
        if param.name == "middle_point_fraction":
            node.middle_point_fraction = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "middle_point_height":
            node.middle_point_height = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "minimum_stair_height":
            node.minimum_stair_height = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "push_off_fraction":
            node.push_off_fraction = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "push_off_position":
            node.push_off_position = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "add_push_off":
            node.add_push_off = param.value
            dynamic_gait_updated = True
        elif param.name == "amount_of_steps":
            node.amount_of_steps = param.get_parameter_value().integer_value
            dynamic_gait_updated = True
        elif param.name == "use_position_queue":
            node.use_position_queue = param.get_parameter_value().bool_value
            position_queue_updated = True
        elif param.name == "add_cybathlon_gaits":
            node.add_cybathlon_gaits = param.get_parameter_value().bool_value
            dynamic_gait_updated = True
        elif param.name == "ankle_buffer":
            node.ankle_buffer = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "hip_buffer":
            node.hip_buffer = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "default_knee_bend":
            node.default_knee_bend = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "hip_x_fraction":
            node.hip_x_fraction = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "upper_body_front_rotation":
            node.upper_body_front_rotation = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "dorsiflexion_at_end_position":
            node.dorsiflexion_at_end_position = param.get_parameter_value().double_value
            dynamic_gait_updated = True

    # Separate update function for dynamic gait to avoid time performance issues
    if dynamic_gait_updated:
        gait_state_machine.update_parameters("dynamic_walk")
    elif position_queue_updated:
        gait_state_machine.update_parameters("dynamic_step")
        gait_state_machine.update_parameters("dynamic_step_and_hold")

    node._logger.info(f"{param.name} set to {param.value}.")

    return SetParametersResult(successful=True)


class GaitNode(Node):
    """Generic node class for gait classes to create subscribers/publishers/parameters on."""

    def __init__(self):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)
        self._logger = self.get_logger().get_child(__class__.__name__)
        self._set_reconfigurable_parameters()

    def _set_reconfigurable_parameters(self) -> None:
        """Initialize all parameters of the gait loader and of the gaits."""
        try:
            self.gait_package = self.get_parameter("gait_package").get_parameter_value().string_value
            self.directory_name = self.get_parameter("gait_directory").get_parameter_value().string_value
            self.early_schedule_duration = self._parse_duration_parameter("early_schedule_duration")
            self.first_subgait_delay = self._parse_duration_parameter("first_subgait_delay")

            # Dynamic gait parameters
            self.middle_point_fraction = self.get_parameter("middle_point_fraction").get_parameter_value().double_value
            self.middle_point_height = self.get_parameter("middle_point_height").get_parameter_value().double_value
            self.use_position_queue = self.get_parameter("use_position_queue").get_parameter_value().bool_value
            self.amount_of_steps = self.get_parameter("amount_of_steps").get_parameter_value().integer_value
            self.minimum_stair_height = self.get_parameter("minimum_stair_height").get_parameter_value().double_value
            self.add_cybathlon_gaits = self.get_parameter("add_cybathlon_gaits").get_parameter_value().bool_value
            self.add_push_off = self.get_parameter("add_push_off").get_parameter_value().bool_value
            self.push_off_fraction = self.get_parameter("push_off_fraction").get_parameter_value().double_value
            self.push_off_position = self.get_parameter("push_off_position").get_parameter_value().double_value

            # IK Solver parameters
            self.ankle_buffer = self.get_parameter("ankle_buffer").get_parameter_value().double_value
            self.hip_buffer = self.get_parameter("hip_buffer").get_parameter_value().double_value
            self.default_knee_bend = self.get_parameter("default_knee_bend").get_parameter_value().double_value
            self.hip_x_fraction = self.get_parameter("hip_x_fraction").get_parameter_value().double_value
            self.upper_body_front_rotation = (
                self.get_parameter("upper_body_front_rotation").get_parameter_value().double_value
            )
            self.dorsiflexion_at_end_position = (
                self.get_parameter("dorsiflexion_at_end_position").get_parameter_value().double_value
            )

        except ParameterNotDeclaredException:
            self._logger.error("Gait node started without the required parameters.")

    def _parse_duration_parameter(self, name: str) -> Duration:
        """Get a duration parameter from the parameter server.

        Returns:
            Duration: duration of the parameter given by name. If param does not exist or is negative, returns zero
        """
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            if value < 0:
                value = 0
            return Duration(seconds=value)
        else:
            return Duration(0)

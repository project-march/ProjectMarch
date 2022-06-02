"""Author: ???."""

import signal
import sys

import rclpy
from march_utility.utilities.duration import Duration
from rcl_interfaces.msg import SetParametersResult
from rclpy import Parameter
from rclpy.executors import MultiThreadedExecutor

from .gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_state_machine import GaitStateMachine
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler

from contextlib import suppress


def sys_exit(*_):
    """Cleanly exit."""
    sys.exit(0)


def main():
    """Starts up the gait selection node with the state machine and scheduler."""
    rclpy.init()

    gait_selection = GaitSelection()
    scheduler = TrajectoryScheduler(gait_selection)
    gait_state_machine = GaitStateMachine(gait_selection, scheduler)
    gait_state_machine.run()
    executor = MultiThreadedExecutor()

    gait_selection.add_on_set_parameters_callback(
        lambda params: parameter_callback(gait_selection, gait_state_machine, params)
    )

    signal.signal(signal.SIGTERM, sys_exit)

    with suppress(KeyboardInterrupt):
        rclpy.spin(gait_selection, executor)

    rclpy.shutdown()


def parameter_callback(
    gait_selection: GaitSelection, gait_state_machine: GaitStateMachine, parameters
) -> SetParametersResult:
    """A callback function that is used to update the parameters that are a part of the gait selection node.

    Since some of these parameters are from the gait_state_machine, some from gait_selection and some from both, this is
    implemented here.

    Args:
        gait_selection (GaitSelection): The current GaitSelection object
        gait_state_machine (GaitStateMachine): The current GaitStateMachine object
        parameters (???): The parameters to update
    Returns:
        SetParametersResult: Whether the callback was successful
    """
    position_queue_updated = False
    gaits_updated = False
    dynamic_gait_updated = False
    for param in parameters:
        if param.name == "balance" and param.type_ == Parameter.Type.BOOL:
            gait_selection._balance_used = param.get_parameter_value().bool_value
        elif param.name == "dynamic_gait" and param.type == Parameter.Type.BOOL:
            gait_selection._dynamic_gait = param.get_parameter_value().bool_value
        elif param.name == "early_schedule_delay" and param.type_ == Parameter.Type.DOUBLE:
            if param.value < 0:
                return SetParametersResult(successful=False)
            gait_selection._early_schedule_duration = Duration(seconds=param.value)
        elif param.name == "first_subgait_delay" and param.type_ == Parameter.Type.DOUBLE:
            if param.value < 0:
                return SetParametersResult(successful=False)
            gait_selection._first_subgait_delay = Duration(seconds=param.value)
        elif param.name == "middle_point_fraction":
            gait_selection.middle_point_fraction = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "middle_point_height":
            gait_selection.middle_point_height = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "minimum_stair_height":
            gait_selection.minimum_stair_height = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "push_off_fraction":
            gait_selection.push_off_fraction = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "push_off_position":
            gait_selection.push_off_position = param.get_parameter_value().double_value
            dynamic_gait_updated = True
        elif param.name == "add_push_off":
            gait_selection.add_push_off = param.value
            dynamic_gait_updated = True
        elif param.name == "amount_of_steps":
            gait_selection.amount_of_steps = param.get_parameter_value().integer_value
            dynamic_gait_updated = True
        elif param.name == "use_position_queue":
            gait_selection.use_position_queue = param.get_parameter_value().bool_value
            position_queue_updated = True
        elif param.name == "gait_package" and param.type_ == Parameter.Type.STRING:
            gait_selection._gait_package = param.value
            gaits_updated = True
        elif param.name == "gait_directory" and param.type_ == Parameter.Type.STRING:
            gait_selection._directory_name = param.value
            gaits_updated = True
        elif param.name == "timer_period" and param.type_ == Parameter.Type.DOUBLE:
            gait_state_machine.timer_period = param.value
            if gait_state_machine.update_timer is not None:
                gait_state_machine.update_timer.destroy()
            gait_state_machine.run()

    # Separate update function for dynamic gait to avoid time performance issues
    if dynamic_gait_updated:
        gait_selection.dynamic_setpoint_gait.update_parameters()
    elif position_queue_updated:
        gait_selection.dynamic_setpoint_gait_step.update_parameter()
        gait_selection.dynamic_setpoint_gait_step_and_hold.update_parameter()
    elif gaits_updated:
        # TODO: Updating the parameters in gait_selection does not work
        gait_selection.update_gaits()
        gait_state_machine._generate_graph()
        gait_selection.get_logger().info("Gaits were updated")

    gait_selection.get_logger().info(f"{param.name} set to {param.value}.")

    return SetParametersResult(successful=True)

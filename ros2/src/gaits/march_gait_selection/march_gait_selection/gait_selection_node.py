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


def sys_exit(*_):
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

    try:
        rclpy.spin(gait_selection, executor)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


def parameter_callback(gait_selection, gait_state_machine, parameters):
    """
    A callback function that is used to update the parameters that are a part of the
    gait selection node. Since some of these parameters are from the
    gait_state_machine, some from gait_selection and some from both, this is
    implemented here.
    :param gait_selection: The current GaitSelection object
    :param gait_state_machine: The current GaitStateMachine
    :param parameters: the parameters to update
    :return: Whether the callback was successful
    """
    gait_selection.get_logger().info("Parameters are updated")
    gaits_updated = False
    for param in parameters:
        if param.name == "balance" and param.type_ == Parameter.Type.BOOL:
            gait_selection._balance_used = param.value
        elif (
            param.name == "early_schedule_delay"
            and param.type_ == Parameter.Type.DOUBLE
        ):
            value = param.value
            if value < 0:
                value = 0
            gait_selection._early_schedule_duration = Duration(seconds=value)
        elif (
            param.name == "first_subgait_delay" and param.type_ == Parameter.Type.DOUBLE
        ):
            value = param.value
            if value < 0:
                value = 0
            gait_selection._first_subgait_delay = Duration(seconds=value)
        elif param.name == "gait_package" and param.type_ == Parameter.Type.STRING:
            gait_selection._gait_package = param.value
            gaits_updated = True
        elif param.name == "gait_directory" and param.type_ == Parameter.Type.STRING:
            gait_selection._directory_name = param.value
            gaits_updated = True
        elif param.name == "timer_period" and param.type_ == Parameter.Type.DOUBLE:
            gait_state_machine.timer_period = param.value
            gait_state_machine.run()

    if gaits_updated:
        gait_selection.update_gaits()
        gait_state_machine._generate_graph()
        gait_selection.get_logger().info("Gaits were updated")

    return SetParametersResult(successful=True)

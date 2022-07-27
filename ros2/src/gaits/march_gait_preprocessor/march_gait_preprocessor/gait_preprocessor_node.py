"""Author: Marten Haitjema, MVII."""

import signal
import sys
import rclpy

from typing import List
from rclpy.parameter import Parameter
from .gait_preprocessor import GaitPreprocessor
from rcl_interfaces.msg import SetParametersResult
from contextlib import suppress


def sys_exit(*_):
    """To shut down the node without an error."""
    rclpy.shutdown()
    sys.exit(0)


def main():
    """To start up the Node."""
    rclpy.init()
    gait_preprocessor = GaitPreprocessor()

    gait_preprocessor.add_on_set_parameters_callback(lambda params: parameter_callback(gait_preprocessor, params))

    signal.signal(signal.SIGTERM, sys_exit)

    with suppress(KeyboardInterrupt):
        rclpy.spin(gait_preprocessor)

    rclpy.shutdown()


def parameter_callback(gait_preprocessor: GaitPreprocessor, parameters: List[Parameter]) -> SetParametersResult:
    """Update parameter of gait_preprocessor and return if this is done successfully.

    Args:
        gait_preprocessor (GaitPreprocessor): gait_preprocessor class.
        parameters (list[Parameter]): List containing the changed parameters.

    Returns:
        SetParametersResult: Whether the parameters were set successfully.
    """
    for param in parameters:
        if param.name == "location_x":
            gait_preprocessor._location_x = param.get_parameter_value().double_value
        elif param.name == "location_y":
            gait_preprocessor._location_y = param.get_parameter_value().double_value
        elif param.name == "location_z":
            gait_preprocessor._location_z = param.get_parameter_value().double_value
        elif param.name == "offset_x":
            gait_preprocessor._offset_x = param.get_parameter_value().double_value
        elif param.name == "offset_y":
            gait_preprocessor._offset_y = param.get_parameter_value().double_value
        elif param.name == "offset_z":
            gait_preprocessor._offset_z = param.get_parameter_value().double_value
        elif param.name == "duration":
            gait_preprocessor._duration = param.get_parameter_value().double_value
        elif param.name == "simulated_deviation":
            gait_preprocessor._simulated_deviation = param.get_parameter_value().double_value
        elif param.name == "deviation_coefficient":
            gait_preprocessor._deviation_coefficient = param.get_parameter_value().double_value
        elif param.name == "midpoint_increase":
            gait_preprocessor._midpoint_increase = param.get_parameter_value().double_value
        elif param.name == "minimum_high_point_ratio":
            gait_preprocessor._minimum_high_point_ratio = param.get_parameter_value().double_value
        elif param.name == "max_deviation":
            gait_preprocessor._max_deviation = param.get_parameter_value().double_value
        elif param.name == "new_midpoint_method":
            gait_preprocessor._new_midpoint_method = param.get_parameter_value().bool_value

        parameter_updated_logger(gait_preprocessor, param)

    return SetParametersResult(successful=True)


def parameter_updated_logger(gait_preprocessor: GaitPreprocessor, param: Parameter):
    """Log which param has been updated to which value."""
    gait_preprocessor._logger.info(f"{param.name} set to {param.value}")

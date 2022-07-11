"""This module contains some errors that are specific to the gaits in the Project March code."""

from typing import Dict


class GaitError(Exception):
    """Base class for exceptions in gait modules.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "An error occurred with a gait module."
        super(GaitError, self).__init__(msg)


class GaitNameNotFoundError(GaitError):
    """Class to raise an error when given gait name does not exist.

    Args:
        gait_name (str): The name of the gait that could not be found.
        msg (str, optional): The error message to display.
    """

    def __init__(self, gait_name, msg=None):
        if msg is None:
            msg = "Could not find gait name: {gait} in map.".format(gait=gait_name)

        super(GaitNameNotFoundError, self).__init__(msg)


class SubgaitNameNotFoundError(GaitError):
    """Class to raise an error when given subgait name does not exist.

    Args:
        subgait_name (str): The name of the subgait that is not recognized within the gait.
        gait_name (str): The name of the gait that could not be found.
        msg (str, optional): The error message to display.
    """

    def __init__(self, subgait_name, gait_name, msg=None):
        if msg is None:
            msg = "Could not find subgait name {subgait} of gait {gait} in map.".format(
                subgait=subgait_name, gait=gait_name
            )

        super(SubgaitNameNotFoundError, self).__init__(msg)


class NonValidGaitContentError(GaitError):
    """Class to raise an error when given gait has incorrect content .

    Args:
        gait_name (str): The name of the gait that could not be found.
        msg (str, optional): The error message to display.
    """

    def __init__(self, gait_name=None, msg=None):
        if msg is None:
            msg = "The given gait: {gn} has incorrect information".format(gn=gait_name)

        super(NonValidGaitContentError, self).__init__(msg)


class SubgaitGraphError(GaitError):
    """Todo: Add docstring."""

    def __init__(self, msg):
        super(SubgaitGraphError, self).__init__(msg)


class TransitionError(GaitError):
    """Class to raise an error when transition between two subgaits has an error.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "Subgaits can not transition"

        super(TransitionError, self).__init__(msg)


class SubgaitInterpolationError(GaitError):
    """Class to raise an error when it was not possible to interpolate between subgaits.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "An error occurred while trying to merge two subgaits."

        super(SubgaitInterpolationError, self).__init__(msg)


class UnknownDimensionsError(Exception):
    """Class to raise an error when the dimension is not within the InterpolationDimensions enum, and therefore unknown.

    Args:
        dimensions: The actual value or object that should have been a value of the InterpolationDimensions enum..
    """

    def __init__(self, dimensions):
        msg = f"Unknown amount of dimensions, should be from InterpolationDimensions enum, but was: {dimensions}"
        super(UnknownDimensionsError, self).__init__(msg)


class WrongRealSenseConfigurationError(Exception):
    """Class to raise an error when there was a mistake in the realsense_gaits.yaml.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg: str = None):
        if msg is None:
            msg = "An error occurred while trying to read out the realsense config."

        super(WrongRealSenseConfigurationError, self).__init__(msg)


class PositionSoftLimitError(Exception):
    """Class to raise an error when joint trajectory will be outside of position soft limits.

    Args:
        joint_name (str): name of the joint
        position (float): position of the joint
        lower_limit (float): lower limit of position soft limits
        upper_limit (float): upper limit of position soft limits
    """

    def __init__(self, joint_name: str, position: float, lower_limit: float, upper_limit: float):
        msg = (
            f"{joint_name} will be outside its soft limits. Position: {position}, soft limits: "
            f"[{lower_limit}, {upper_limit}]."
        )

        super(PositionSoftLimitError, self).__init__(msg)


class VelocitySoftLimitError(Exception):
    """Class to raise an error when joint trajectory will be outside of velocity soft limits.

    Args:
        joint_name (str): name of the joint that is outside of velocity limit
        velocity (float): velocity of the joint
        limit (float): velocity limit of the joint
    """

    def __init__(self, joint_name: str, velocity: float, limit: float):
        msg = f"{joint_name} will be outside of velocity limits, velocity: {velocity}, velocity limit: {limit}."

        super(VelocitySoftLimitError, self).__init__(msg)


class WrongStartPositionError(Exception):
    """Exception for when the exo is not starting from the correct position.

    Mainly raised if an error when the previous subgait failed and dynamic gait is selected again
    without the exo being in home stand.

    Args:
        correct_position (Dict[str, float]): position the exo should start in.
        wrong_position (Dict[str, float]): current position of the exo.
    """

    def __init__(self, correct_position: Dict[str, float], wrong_position: Dict[str, float]):
        msg = f"Gait can only be executed from position {correct_position}, current position is {wrong_position}."

        super(WrongStartPositionError, self).__init__(msg)

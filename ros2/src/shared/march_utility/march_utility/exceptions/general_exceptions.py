"""This module contains some errors that are specific to the Project March code."""
from __future__ import annotations


class PackageNotFoundError(Exception):
    """Class to raise an error when a ros1 package cannot be found.

    Args:
        package_name (str): The package name which is not found by rospkg.RosPack().get_path().
        msg (str, optional) a custom error message to return. Default is `None`.
    """

    def __init__(self, package_name, msg=None):
        if msg is None:
            msg = "Package: {fp} could not be found.".format(fp=package_name)

        super(PackageNotFoundError, self).__init__(msg)


class MsgTypeError(Exception):
    """Class to raise an error when an non msg type is added to a message.

    Args:
        msg (str, optional): The error message to display. Default is `None`.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "A non msg type (defined in shared resources) was added to a ROS-message"

        super(MsgTypeError, self).__init__(msg)


class SideSpecificationError(Exception):
    """Class to raise an error when a foot ('right' or 'left') has to be specified but this did not happen.

    Args:
        msg (str, optional): The error message to display. Default is `None`.
    """

    def __init__(self, foot, msg=None):
        if msg is None:
            msg = "An incorrect side was supplied. Must be a either Side.left or Side.right, but was {foot}.".format(
                foot=foot
            )

        super(SideSpecificationError, self).__init__(msg)


class IncorrectCoordinateError(Exception):
    """Class to raise an error when the coordinates of a position are incorrect.

    Args:
        msg (str, optional): The error message to display. Default is `None`.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "The keys of a position or velocity dictionary should be ['x', 'y', 'z'], but were different."

        super(IncorrectCoordinateError, self).__init__(msg)


class WeightedAverageError(Exception):
    """Class to raise an error when a weighted average cannot be computed.

    Args:
        msg (str, optional): The error message to display. Default is `None`.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "The calculation of the weighted average cannot be executed safely."

        super(WeightedAverageError, self).__init__(msg)


class InconsistentDigitsError(Exception):
    """Class to raise an error when precision or digits are not consistent where they should be.

    Args:
        msg (str, Optional): The custom error message to return.
        number1 (float, Optional): If `number1` and `number2` are specified, then
            they are included in the default error message.
        number2 (float, Optional): If `number1` and `number2` are specified, then
            they are included in the default error message.
    """

    def __init__(self, msg: str = None, number1: float = None, number2: float = None) -> None:
        if msg is None:
            msg = "Two numbers which which should have the same number of digits but do not were supplied."
            if number1 is not None and number2 is not None:
                msg += f"The numbers are: {number1} and {number2}"

        super(InconsistentDigitsError, self).__init__(msg)

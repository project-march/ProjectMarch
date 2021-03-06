"""This module contains some errors that are specific to the Project March code."""
from __future__ import annotations
from march_utility.utilities.side import Side


class PackageNotFoundError(Exception):
    """Class to raise an error when a ros package cannot be found."""

    def __init__(self, package_name: str, msg: str = None) -> None:
        """
        Initialize PackageNotFoundError.

        :param package_name: The package that was not found.
        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = f"Package: {package_name} could not be found."

        super(PackageNotFoundError, self).__init__(msg)


class MsgTypeError(Exception):
    """Class to raise an error when an non msg type is added to a message."""

    def __init__(self, msg: str = None) -> None:
        """
        Initialize MsgTypeError.

        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = "A non msg type (defined in shared resources) was added to a ROS-message"

        super(MsgTypeError, self).__init__(msg)


class SideSpecificationError(Exception):
    """Class to raise an error when wrong side ('right' or 'left') was specified."""

    def __init__(self, foot: Side, msg: str = None) -> None:
        """
        Initialize side specification error.

        :param foot: The foot which was wrongly specified.
        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = (
                f"An incorrect side was supplied. Must be a either Side.left or "
                f"Side.right, but was {foot}."
            )

        super(SideSpecificationError, self).__init__(msg)


class IncorrectCoordinateError(Exception):
    """Class to raise an error when the coordinates of a position are incorrect."""

    def __init__(self, msg: str = None) -> None:
        """
        Initialize IncorrectCoordinateError.

        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = (
                "The keys of a position or velocity dictionary should be "
                "['x', 'y', 'z'], but were different."
            )

        super(IncorrectCoordinateError, self).__init__(msg)


class WeightedAverageError(Exception):
    """Class to raise an error when a weighted average cannot be computed."""

    def __init__(self, msg: str = None) -> None:
        """
        Initialize WeightedAverageError.

        :param msg: Optional, a custom error message to return.
        """
        if msg is None:
            msg = "The calculation of the weighted average cannot be executed safely."

        super(WeightedAverageError, self).__init__(msg)


class InconsistentDigitsError(Exception):
    def __init__(
        self, msg: str = None, number1: float = None, number2: float = None
    ) -> None:
        """
        Class to raise an error when precision or digits are not consistent where they should be.

        :param msg: Optional, a custom error message to return
        :param number1: Optional, if both number1 and number2 are specified
        they are included in the default error message
        :param number2: Optional, if both number1 and number2 are specified
        they are included in the default error message
        """
        if msg is None:
            msg = "Two numbers which which should have the same number of digits but do not were supplied."
            if number1 is not None and number2 is not None:
                msg += f"The numbers are: {number1} and {number2}"

        super(InconsistentDigitsError, self).__init__(msg)

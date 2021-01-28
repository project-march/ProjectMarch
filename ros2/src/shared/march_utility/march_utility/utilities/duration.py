"""This module contains the duration class, which is an extension of the rclpy Duration class."""
from __future__ import annotations

import math
from typing import Union, Any

from march_utility.utilities.utility_functions import weighted_average_floats
from rclpy.duration import Duration as ROSDuration

NSEC_DIGITS = 9
NSEC_IN_SEC = 1000000000


class Duration(ROSDuration):
    """The duration class is an extension of the rclpy Duration class."""

    def __init__(self, seconds: float = 0.0, nanoseconds: float = 0.0):
        super().__init__(seconds=seconds, nanoseconds=nanoseconds)

    @property
    def seconds(self) -> float:
        """Convert the nanoseconds to seconds."""
        return self.nanoseconds / NSEC_IN_SEC

    def __round__(self, n: int = None) -> Duration:
        """Round the nanoseconds as if it were seconds.

        This round works as followings:
        Say self.seconds = 1.25 and you want to round to 1 digit (n=1)
        Then self.nanoseconds = 1250000000
        Since n=1, n_pow becomes 10^8.
        Then self.nanoseconds / n_pow becomes 12.5 and (12.5) = 13.
        After multiplying again by n_pow we get 1.3 * 10^9 nanoseconds, which is equal
        to 1,3 seconds.

        :param n Number of decimals to round to
        :return Returns a new duration, with the rounded amount of seconds.
        """
        n_pow = math.pow(10, NSEC_DIGITS - n)
        rounded_nanoseconds = round(self.nanoseconds / n_pow) * n_pow
        return Duration(nanoseconds=rounded_nanoseconds)

    def weighted_average(self, other: Duration, parameter: float) -> Duration:
        """Take the weight average of another duration.

        :param other: Other duration
        :param parameter: parameter to use in taking weighted average

        :return: Returns the weighted duration
        """
        if not isinstance(other, Duration):
            raise TypeError(
                f"Weighted average expectes other to be a Duration, but got a {type(other)}"
            )
        return Duration(
            nanoseconds=weighted_average_floats(
                self.nanoseconds, other.nanoseconds, parameter
            )
        )

    @classmethod
    def from_ros_duration(cls, duration: ROSDuration) -> Duration:
        """Create a Duration from its Parent class.

        :param duration: ROSDuration
        """
        return cls(nanoseconds=duration.nanoseconds)

    def __add__(self, other: Duration) -> Duration:
        """Add a duration.

        :param other: Duration to add

        :return Returns the result of the addition
        """
        return Duration(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __sub__(self, other: Duration) -> Duration:
        """Subtract a duration.

        :param other: Duration to subtract

        :return Returns the result of the subtraction
        """
        return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __mul__(self, other: Duration) -> Duration:
        """Multiply the duration by a numeric value.

        :param other: Value to multiply by

        :return Returns the result of the multiplication
        """
        if isinstance(other, (int, float)):
            return Duration(nanoseconds=self.nanoseconds * other)
        raise TypeError(f"Expected numerical value, bot got type: {type(other)}")

    def __truediv__(
        self, other: Union[Duration, int, float]
    ) -> Union[Duration, int, float]:
        """Divide the duration by some other value.

        If this value is numeric, a Duration is returned.
        If this value is another Duration, a numeric value is returned.

        :param other: What to divide by

        :return Returns the result of the division
        """
        if isinstance(other, (int, float)):
            return Duration(nanoseconds=self.nanoseconds / other)
        if isinstance(other, Duration):
            return self.nanoseconds / other.nanoseconds
        raise TypeError(
            f"Expected numerical value or Duration, bot got type: {type(other)}"
        )

    def __hash__(self):
        """Return a hash based on the nanoseconds attribute."""
        return hash(self.nanoseconds)

    def __deepcopy__(self, _: Any) -> Duration:
        """Create a deepcopy of the Duration."""
        return Duration(nanoseconds=self.nanoseconds)

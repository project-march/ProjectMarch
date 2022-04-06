"""This module contains the duration class, which is an extension of the rclpy Duration class."""
from __future__ import annotations

import math
from typing import Union, Any

from march_utility.utilities.utility_functions import weighted_average_floats
from rclpy.duration import Duration as ROSDuration

NSEC_DIGITS = 9
NSEC_IN_SEC = 1000000000


class Duration(ROSDuration):
    """The duration class is an extension of the rclpy Duration class.

    Everytime 'Duration' is mentioned as type we mean this object,
    if we mean the rclpy Duration we use 'rclpy.Duration' or 'ROSDuration'.
    """

    def __init__(self, seconds: float = 0.0, nanoseconds: float = 0.0):
        super().__init__(seconds=seconds, nanoseconds=nanoseconds)

    @property
    def seconds(self) -> float:
        """Convert the nanoseconds to seconds."""
        return self.nanoseconds / NSEC_IN_SEC

    def __round__(self, n: int = None) -> Duration:
        """Round the nanoseconds as if it were seconds.

        Example:
            Say self.seconds = 1.25 and you want to round to 1 digit (n=1)
            Then self.nanoseconds = 1250000000
            Since n=1, n_pow becomes 10^8.
            Then self.nanoseconds / n_pow becomes 12.5 and (12.5) = 13.
            After multiplying again by n_pow we get 1.3 * 10^9 nanoseconds, which is equal
            to 1,3 seconds.

        Args:
             n (int): Number of decimals to round to.

        Returns:
             Duration. A new duration, with the rounded amount of seconds.
        """
        n_pow = math.pow(10, NSEC_DIGITS - n)
        rounded_nanoseconds = round(self.nanoseconds / n_pow) * n_pow
        return Duration(nanoseconds=rounded_nanoseconds)

    def weighted_average(self, other: Duration, parameter: float) -> Duration:
        """Take the weight average of another duration.

        Args:
            other (Duration): Other duration.
            parameter (float): Parameter to use in taking weighted average.

        Returns:
             Duration. The weighted duration.
        """
        if not isinstance(other, Duration):
            raise TypeError(f"Weighted average expectes other to be a Duration, but got a {type(other)}")
        return Duration(nanoseconds=weighted_average_floats(self.nanoseconds, other.nanoseconds, parameter))

    @classmethod
    def from_ros_duration(cls, duration: ROSDuration) -> Duration:
        """Create a Duration from its Parent class.

        Args:
            duration (rclpy.duration): A ROSDuration to create this duration object for.

        Returns:
            Duration. An instance of THIS duration object.
        """
        return cls(nanoseconds=duration.nanoseconds)

    def __add__(self, other: Duration) -> Duration:
        """Add a duration.

        Args:
            other (Duration): Duration to add.

        Returns:
             Duration. The result of the addition
        """
        return Duration(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __sub__(self, other: Duration) -> Duration:
        """Subtract a duration.

        Args:
            other (Duration): Duration to subtract.

        Returns:
             Duration. The result of the subtraction.
        """
        return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __mul__(self, other: Union[Duration, int, float]) -> Duration:
        """Multiply the duration by a numeric value.

        Args:
            other (Duration, int, float): Value to multiply by.

        Returns:
             Duration. The result of the multiplication.

        Raises:
            TypeError: If it is not multiplied by an `int`, `float` or other `Duration`.
        """
        if isinstance(other, (int, float)):
            return Duration(nanoseconds=self.nanoseconds * other)
        if isinstance(other, Duration):
            return Duration(nanoseconds=(self.nanoseconds * other.nanoseconds))
        raise TypeError(f"Expected numerical value or Duration, bot got type: {type(other)}")

    def __truediv__(self, other: Union[Duration, int, float]) -> Union[Duration, float]:
        """Divide the duration by some other value.

        If this value is numeric, a Duration is returned.
        If this value is another Duration, a numeric value is returned.

        Args:
            other (Duration, int, float): What to divide by.

        Returns:
            The result of the division in either `float` or `Duration` depending on the type of the `other` object.
                Duration.  If `other` is an instance of Duration.
                float.  If `other` is an instance of `int` or `float`.
        """
        if isinstance(other, (int, float)):
            return Duration(nanoseconds=self.nanoseconds / other)
        if isinstance(other, Duration):
            return self.nanoseconds / other.nanoseconds
        raise TypeError(f"Expected numerical value or Duration, bot got type: {type(other)}")

    def __hash__(self):
        """Return a hash based on the nanoseconds attribute."""
        return hash(self.nanoseconds)

    def __deepcopy__(self, _: Any) -> Duration:
        """Create a deepcopy of the Duration."""
        return Duration(nanoseconds=self.nanoseconds)

    def __str__(self):
        """The string of this duration is in seconds rounded to 4 decimals."""
        return f"{round(self.seconds, 4)}s"

    def __abs__(self):
        """Sets nanoseconds to be positive."""
        return Duration(nanoseconds=abs(self.nanoseconds))

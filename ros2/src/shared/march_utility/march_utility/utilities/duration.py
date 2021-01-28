from __future__ import annotations

import math
from copy import deepcopy

from march_utility.utilities.utility_functions import weighted_average_floats
from rclpy.duration import Duration as ROSDuration

NSEC_DIGITS = 9
NSEC_TO_SEC = math.pow(10, -NSEC_DIGITS)


class Duration(ROSDuration):
    def __init__(self, seconds: float = 0.0, nanoseconds: float = 0.0):
        super().__init__(seconds=seconds, nanoseconds=nanoseconds)

    @property
    def seconds(self):
        return self.nanoseconds * NSEC_TO_SEC

    def __round__(self, n=None) -> Duration:
        n_pow = math.pow(10, NSEC_DIGITS - n)
        rounded_nanoseconds = round(self.nanoseconds / n_pow) * n_pow
        return Duration(nanoseconds=rounded_nanoseconds)

    def weighted_average(self, other: Duration, parameter: float) -> Duration:
        if not isinstance(other, Duration):
            raise TypeError(f"Weighted average expectes other to be a Duration, but got a {type(other)}")
        return Duration(nanoseconds=weighted_average_floats(self.nanoseconds, other.nanoseconds, parameter))

    @classmethod
    def from_ros_duration(cls, duration: ROSDuration):
        return cls(nanoseconds=duration.nanoseconds)

    def __add__(self, other):
        return Duration(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __sub__(self, other):
        return Duration(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Duration(nanoseconds=self.nanoseconds * other)
        raise TypeError(
            f"Expected numerical value or Duration, bot got type: {type(other)}")

    def __truediv__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return Duration(nanoseconds=self.nanoseconds / other)
        if isinstance(other, Duration):
            return self.nanoseconds / other.nanoseconds
        raise TypeError(f"Expected numerical value or Duration, bot got type: {type(other)}")

    def __hash__(self):
        return hash(self.nanoseconds)

    def __deepcopy__(self, memo):
        return Duration(nanoseconds=self.nanoseconds)
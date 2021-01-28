from __future__ import annotations

import math

from march_utility.utilities.utility_functions import weighted_average_floats
from rclpy.duration import Duration as ROSDuration

NSEC_DIGITS = 9
NSEC_TO_SEC = math.pow(10, -NSEC_DIGITS)


class CustomDuration(ROSDuration):
    def __init__(self, seconds: float = 0.0, nanoseconds: float = 0.0):
        super().__init__(seconds=seconds, nanoseconds=nanoseconds)

    @property
    def seconds(self):
        return self.nanoseconds * NSEC_TO_SEC

    def __round__(self, n=None) -> CustomDuration:
        n_pow = math.pow(10, NSEC_DIGITS - n)
        rounded_nanoseconds = round(self.nanoseconds / n_pow) * n_pow
        return CustomDuration(nanoseconds=rounded_nanoseconds)

    def weighted_average(self, other: CustomDuration, parameter: float) -> CustomDuration:
        if not isinstance(other, CustomDuration):
            raise TypeError(f"Weighted average expectes other to be a CustomDuration, but got a {type(other)}")
        return CustomDuration(nanoseconds=weighted_average_floats(self.nanoseconds, other.nanoseconds, parameter))

    def __add__(self, other):
        return CustomDuration(nanoseconds=self.nanoseconds + other.nanoseconds)

    def __sub__(self, other):
        return CustomDuration(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __mul__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return CustomDuration(nanoseconds=self.nanoseconds * other)
        raise TypeError(
            f"Expected numerical value or CustomDuration, bot got type: {type(other)}")

    def __truediv__(self, other):
        if isinstance(other, int) or isinstance(other, float):
            return CustomDuration(nanoseconds=self.nanoseconds / other)
        if isinstance(other, CustomDuration):
            return self.nanoseconds / other.nanoseconds
        raise TypeError(f"Expected numerical value or CustomDuration, bot got type: {type(other)}")


    def __rtruediv__(self, other):
        if not (isinstance(other, int) or isinstance(other, float)):
            raise TypeError(f"Expected numerical value, bot got type: {type(other)}")
        return other / self.seconds

    def __hash__(self):
        return hash(self.nanoseconds)
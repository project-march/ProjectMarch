"""This module contains the Setpoint class used for defining gaits."""

from __future__ import annotations

from typing import Optional

# Use this factor when calculating velocities to keep the calculations within the range of motion
# See IK confluence page https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    weighted_average_floats,
)

VELOCITY_SCALE_FACTOR = 0.001


class Setpoint:
    """Base class to define the setpoints of a subgait."""

    digits = 8

    def __init__(
        self, time: Duration, position: float, velocity: Optional[float] = None
    ) -> None:
        """
        Initialize a setpoint.

        :param time: The time within the subgait, in nanoseconds.
        :param position: The position (angle) of the joint.
        :param velocity: The velocity of the joint.
        """
        self._time = round(
            time, self.digits
        )  # https://github.com/python/mypy/issues/8213
        self._position = round(position, self.digits)
        if velocity is not None:
            self._velocity: Optional[float] = round(velocity, self.digits)
        else:
            self._velocity = None

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, time: float):
        self._time = round(time, self.digits)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position: float):
        self._position = round(position, self.digits)

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, velocity: float):
        self._velocity = round(velocity, self.digits)

    def __repr__(self):
        if self.velocity is not None:
            return (
                f"Time: {self.time.nanoseconds!s}, Position: {self.position!s}, Velocity:"
                f" {self.velocity!s}"
            )
        else:
            return (
                f"Time: {self.time!s}, Position: {self.position!s}, Velocity: Not "
                f"specified"
            )

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return (
                self.time == other.time
                and self.position == other.position
                and self.velocity == other.velocity
            )
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    @staticmethod
    def interpolate_setpoints(
        base_setpoint: Setpoint, other_setpoint: Setpoint, parameter: float
    ) -> Setpoint:
        """Linearly interpolate two setpoints.

        :param base_setpoint:
            base setpoint, return value if parameter is zero
        :param other_setpoint:
            other setpoint, return value if parameter is one
        :param parameter:
            parameter for linear interpolation, 0 <= parameter <= 1
        :return:
            The interpolated setpoint
        """
        time = base_setpoint.time.weighted_average(other_setpoint.time, parameter)
        position = weighted_average_floats(
            base_setpoint.position, other_setpoint.position, parameter
        )
        velocity = weighted_average_floats(
            base_setpoint.velocity, other_setpoint.velocity, parameter
        )
        return Setpoint(time, position, velocity)

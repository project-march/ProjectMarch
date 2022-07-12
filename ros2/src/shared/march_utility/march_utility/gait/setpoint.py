"""This module contains the Setpoint class used for defining gaits."""

from __future__ import annotations

from typing import Optional, Union

# Use this factor when calculating velocities to keep the calculations within the range of motion
# See IK confluence page https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    weighted_average_floats,
)

VELOCITY_SCALE_FACTOR = 0.001


class Setpoint:
    """Base class to define the setpoints of a subgait.

    Args:
        time (Duration): The time within the subgait.
        position (float): The position (angle) of the joint.
        velocity (float, Optional): The velocity of the joint. Default is `None`.
    """

    digits = 4

    def __init__(self, time: Duration, position: float, velocity: Optional[float] = None) -> None:
        self._time = round(time, self.digits)  # https://github.com/python/mypy/issues/8213
        self._position = round(position, self.digits)
        if velocity is not None:
            self._velocity: Optional[float] = round(velocity, self.digits)
        else:
            self._velocity = None

    @property
    def time(self) -> Duration:
        """Duration. Returns the time of the setpoint within the gait."""
        return self._time

    @time.setter
    def time(self, time: Union[Duration, int, float]):
        if isinstance(time, (int, float)):
            self._time = round(Duration(nanoseconds=time))
        elif isinstance(time, Duration):
            self._time = round(time, self.digits)
        else:
            raise TypeError(f"Expected numerical value or Duration, bot got type: {type(time)}")

    @property
    def position(self) -> float:
        """float. Returns the angle of the joint in this setpoint."""
        return self._position

    @position.setter
    def position(self, position: float):
        self._position = round(position, self.digits)

    @property
    def velocity(self) -> Optional[float]:
        """float. Returns the velocity of the joint in this setpoint."""
        return self._velocity

    @velocity.setter
    def velocity(self, velocity: float):
        self._velocity = round(velocity, self.digits)

    def __repr__(self):
        """str. The representation of a setpoint instance."""
        if self.velocity is not None:
            return f"Time: {self.time!s}, Position: {self.position!s}, Velocity: {self.velocity!s}"
        else:
            return f"Time: {self.time!s}, Position: {self.position!s}, Velocity: Not specified"

    def __eq__(self, other) -> bool:
        """bool. Checks if the other object is an instance of setpoint and has the same values."""
        if isinstance(other, self.__class__):
            return self.time == other.time and self.position == other.position and self.velocity == other.velocity
        else:
            return False

    @staticmethod
    def interpolate_setpoints(base_setpoint: Setpoint, other_setpoint: Setpoint, parameter: float) -> Setpoint:
        """Linearly interpolate two setpoints.

        Args:
            base_setpoint (Setpoint): The base setpoint to interpolate from.
            other_setpoint (Setpoint): The other setpoint to interpolate to.
            parameter (float): The parameter to use for interpolation. Should be 0 <= parameter <= 1

        Returns:
            Setpoint. The interpolated setpoint:
                * `base_setpoint`, if `parameter` == 0.
                * `other_setpoint`, if `parameter` == 0.
        """
        time = base_setpoint.time.weighted_average(other_setpoint.time, parameter)
        position = weighted_average_floats(base_setpoint.position, other_setpoint.position, parameter)
        velocity = weighted_average_floats(base_setpoint.velocity, other_setpoint.velocity, parameter)
        return Setpoint(time, position, velocity)

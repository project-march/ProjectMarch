"""This module contains the Setpoint class used for defining gaits."""

from __future__ import annotations

from typing import Optional

# Use this factor when calculating velocities to keep the calculations within the range of motion
# See IK confluence page https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    get_joint_names_for_inverse_kinematics,
    weighted_average_floats,
)

VELOCITY_SCALE_FACTOR = 0.001
JOINT_NAMES_IK = get_joint_names_for_inverse_kinematics()


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

    @classmethod
    def calculate_next_positions_joint(cls, setpoint_dic: dict) -> dict:
        """
        Calculate the position of the joints a moment later.

        Calculates using the approximation:
        next_position = position + current_velocity * time_difference
        :param setpoint_dic: A dictionary of setpoints with positions and velocities
        :return: A dictionary with the positions of the joints 1 / VELOCITY_SCALE_FACTOR
        seconds later
        """
        next_positions = {}
        for joint in JOINT_NAMES_IK:
            if joint not in setpoint_dic:
                raise KeyError(f"Setpoint_dic is missing joint {joint}")
            else:
                next_positions[joint] = cls(
                    setpoint_dic[joint].time + Duration(seconds=VELOCITY_SCALE_FACTOR),
                    setpoint_dic[joint].position
                    + setpoint_dic[joint].velocity * VELOCITY_SCALE_FACTOR,
                )

        return next_positions

    def add_joint_velocity_from_next_angle(self, next_state: Setpoint) -> None:
        """Calculate the joint velocities given a current position and a next position.

        Calculates using the approximation:
        next_position = position + current_velocity * time_difference

        :param self: A Setpoint object with no velocity
        :param next_state: A Setpoint with the positions a moment later

        :return: The joint velocities of the joints on the specified side
        """
        self.velocity = (next_state.position - self.position) / (
            next_state.time - self.time
        ).seconds

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

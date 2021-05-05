from typing import Optional

from march_utility.utilities.utility_functions import (
    get_joint_names_for_inverse_kinematics,
)
from march_utility.utilities.duration import Duration

from .setpoint import Setpoint

VELOCITY_SCALE_FACTOR = 0.001
JOINT_NAMES_IK = get_joint_names_for_inverse_kinematics()


class CalculationSetpoint(Setpoint):
    """Base class to define setpoints used in calculations which do not round."""

    def __init__(
        self, time: Duration, position: float, velocity: Optional[float] = None
    ) -> None:
        """
        Initialize a setpoint.

        :param time: The time within the subgait, in nanoseconds.
        :param position: The position (angle) of the joint.
        :param velocity: The velocity of the joint.
        """
        self._time = time
        self._position = position
        if velocity is not None:
            self._velocity: Optional[float] = velocity
        else:
            self._velocity = None

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, time: float):
        self._time = time

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position: float):
        self._position = position

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, velocity: float):
        self._velocity = velocity

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

    def to_normal_setpoint(self):
        return Setpoint(self.time, self.position, self.velocity)

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

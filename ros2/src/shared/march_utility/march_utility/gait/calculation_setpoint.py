from typing import Optional
from march_utility.utilities.duration import Duration

from .setpoint import Setpoint
from ..utilities.utility_functions import get_joint_names_from_urdf

VELOCITY_SCALE_FACTOR = 0.001


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
    def time(self) -> Duration:
        """Return the time of the setpoint"""
        return self._time

    @time.setter
    def time(self, time: float) -> None:
        self._time = time

    @property
    def position(self) -> float:
        """Return the position of the setpoint"""
        return self._position

    @position.setter
    def position(self, position: float) -> None:
        self._position = position

    @property
    def velocity(self) -> float:
        """Return the velocity of the setpoint"""
        return self._velocity

    @velocity.setter
    def velocity(self, velocity: float) -> None:
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
        for joint in setpoint_dic.keys():
            next_positions[joint] = cls(
                setpoint_dic[joint].time + Duration(seconds=VELOCITY_SCALE_FACTOR),
                setpoint_dic[joint].position
                + setpoint_dic[joint].velocity * VELOCITY_SCALE_FACTOR,
            )

        return next_positions

    def to_normal_setpoint(self) -> Setpoint:
        """Creates a normal setpoint from the calculation setpoint."""
        return Setpoint(self.time, self.position, self.velocity)

    def add_joint_velocity_from_next_angle(self, next_state: Setpoint) -> None:
        """Calculate the setpoint velocitiy given a setpoint a moment later.

        Calculates using the approximation:
        next_position = position + current_velocity * time_difference

        :param self: A Setpoint object to which the velocity has to be added
        :param next_state: A Setpoint with the position a moment later
        """
        self.velocity = (next_state.position - self.position) / (
            next_state.time - self.time
        ).seconds

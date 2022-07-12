"""Author: MARCH IV, V, VI, VII."""
import sys
from march_utility.utilities.duration import Duration

from .setpoint import Setpoint

VELOCITY_SCALE_FACTOR = 0.001


class CalculationSetpoint(Setpoint):
    """Base class to define setpoints used in calculations which do not round."""

    digits = sys.maxsize

    @classmethod
    def calculate_next_positions_joint(cls, setpoint_dic: dict) -> dict:
        """Calculate the position of the joints a moment (`VELOCITY_SCALE_FACTOR` seconds) later.

        Calculates using the approximation:
            next_position = position + current_velocity * time_difference

        Args:
            setpoint_dic (dict): A dictionary of setpoints with positions and velocities.


        Returns:
            dict. A dictionary with the positions of the joint 1 / `VELOCITY_SCALE_FACTOR` seconds later.
        """
        next_positions = {}
        for joint in setpoint_dic.keys():
            next_positions[joint] = cls(
                setpoint_dic[joint].time + Duration(seconds=VELOCITY_SCALE_FACTOR),
                setpoint_dic[joint].position + setpoint_dic[joint].velocity * VELOCITY_SCALE_FACTOR,
            )

        return next_positions

    def to_normal_setpoint(self) -> Setpoint:
        """Creates a normal setpoint from the calculation setpoint."""
        return Setpoint(self.time, self.position, self.velocity)

    def add_joint_velocity_from_next_angle(self, next_state: Setpoint) -> None:
        """Calculate the setpoint velocitiy given a setpoint duration later.

        Calculates using the approximation:
            next_position = position + current_velocity * time_difference

        Args:
            next_state (Setpoint): A Setpoint with the position a duration later.
        """
        self.velocity = (next_state.position - self.position) / (next_state.time - self.time).seconds

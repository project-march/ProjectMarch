"""Author: Marten Haitjema, MVII."""

from numpy import deg2rad
from dataclasses import dataclass


@dataclass()
class IKSolverParameters:
    """Dataclass to contain all reconfigurable parameters of the IK solver."""

    ankle_buffer: float
    hip_buffer: float
    default_knee_bend: float
    hip_x_fraction: float
    upper_body_front_rotation: float
    dorsiflexion_at_end_position: float

    @property
    def ankle_buffer_degrees(self) -> float:
        """Returns the ankle buffer in degrees."""
        return self.ankle_buffer

    @property
    def ankle_buffer_radians(self) -> float:
        """Returns the ankle buffer in radians."""
        return deg2rad(self.ankle_buffer)

    @property
    def hip_buffer_degrees(self) -> float:
        """Returns the hip buffer in degrees."""
        return self.hip_buffer

    @property
    def hip_buffer_radians(self) -> float:
        """Returns the hip buffer in radians."""
        return deg2rad(self.hip_buffer)

    @property
    def default_knee_bend_degrees(self) -> float:
        """Returns the default knee in bend in degrees."""
        return self.default_knee_bend

    @property
    def default_knee_bend_radians(self) -> float:
        """Returns the default knee bend in radians."""
        return deg2rad(self.default_knee_bend)

    @property
    def upper_body_front_rotation_degrees(self) -> float:
        """Returns the upper body front rotation in degrees."""
        return self.upper_body_front_rotation

    @property
    def upper_body_front_rotation_radians(self) -> float:
        """Returns the upper body front in radians."""
        return deg2rad(self.upper_body_front_rotation)

    @property
    def dorsiflexion_at_end_position_degrees(self) -> float:
        """Returns the dorsiflexion at end position in degrees."""
        return self.dorsiflexion_at_end_position

    @property
    def dorsiflexion_at_end_position_radians(self) -> float:
        """Returns the dorsiflexion at end position in radians."""
        return deg2rad(self.dorsiflexion_at_end_position)

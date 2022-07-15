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

    @property
    def ankle_buffer_degrees(self) -> float:
        return self.ankle_buffer

    @property
    def ankle_buffer_radians(self) -> float:
        return deg2rad(self.ankle_buffer)

    @property
    def hip_buffer_degrees(self) -> float:
        return self.hip_buffer

    @property
    def hip_buffer_radians(self) -> float:
        return deg2rad(self.hip_buffer)

    @property
    def default_knee_bend_degrees(self) -> float:
        return self.default_knee_bend

    @property
    def default_knee_bend_radians(self) -> float:
        return deg2rad(self.default_knee_bend)

    @property
    def upper_body_front_rotation_degrees(self) -> float:
        return self.upper_body_front_rotation

    @property
    def upper_body_front_rotation_radians(self) -> float:
        return deg2rad(self.upper_body_front_rotation)

from scipy.interpolate import CubicSpline
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration

NANOSECONDS_TO_SECONDS = 1000000000


class DynamicJointTrajectory:
    """Class that performs interpolation between given list of setpoints.

    :param setpoints: A list containing setpoints for a given joint.
    :type setpoints: list
    """

    def __init__(self, setpoints):
        self.setpoints = setpoints
        self._interpolate_setpoints()

    def _get_setpoints_unzipped(self):
        """Returns a list of time, position and velocity."""
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def _interpolate_setpoints(self):
        """Uses a CubicSpline with velocity boundary conditions to create interpolator objects for
        position and velocity."""
        duration, position, velocity = self._get_setpoints_unzipped()
        boundary_condition = ((1, velocity[0]), (1, velocity[-1]))
        yi = []
        time = []

        for i in range(len(duration)):
            yi.append(position[i])
            time.append(duration[i].nanoseconds / NANOSECONDS_TO_SECONDS)

        self.interpolated_position = CubicSpline(time, yi, bc_type=boundary_condition)
        self.interpolated_velocity = self.interpolated_position.derivative()

    def get_interpolated_setpoint(self, time) -> Setpoint:
        """Computes a Setpoint instance with the given time and the interpolated
        position and velocity at this time.

        :param time: Time at which the setpoint will be set.
        :type time: float

        :returns: A setpoint with the given time and the position and velocity at this time.
        :rtype: Setpoint class instance
        """
        return Setpoint(
            Duration(time),
            float(self.interpolated_position(time)),
            float(self.interpolated_velocity(time)),
        )

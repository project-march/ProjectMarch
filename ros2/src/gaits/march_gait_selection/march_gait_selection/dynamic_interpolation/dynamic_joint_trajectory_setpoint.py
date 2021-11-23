from scipy.interpolate import CubicSpline
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration


class DynamicJointTrajectorySetpoint:

    setpoint_class = Setpoint

    def __init__(self, setpoints):
        self.setpoints = setpoints

    def get_setpoints_unzipped(self):
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def interpolate_setpoints(self):
        duration, position, velocity = self.get_setpoints_unzipped()
        boundary_condition = ((1, velocity[0]), (1, velocity[-1]))
        yi = []
        time = []

        for i in range(len(duration)):
            yi.append(position[i])
            time.append(duration[i].nanoseconds / 1000000000)

        self.interpolated_position = CubicSpline(time, yi, bc_type=boundary_condition)
        self.interpolated_velocity = self.interpolated_position.derivative()

    def get_interpolated_setpoint(self, time):
        return self.setpoint_class(
            Duration(time),
            float(self.interpolated_position(time)),
            float(self.interpolated_velocity(time)),
        )

from march_rqt_gait_generator.model.Limits import Limits
from march_rqt_gait_generator.model.Setpoint import Setpoint


class Joint:

    def __init__(self, name, limits, setpoints):
        self.name = name
        self.limits = limits
        self.setpoints = setpoints
        self.interpolatedSetpoints = self.interpolate_setpoints()

    def get_interpolated_value(self, time):
        for i in range(0, len(self.interpolatedSetpoints)):
            if self.interpolatedSetpoints[i] > time:
                return self.interpolatedSetpoints[i - 1]

    def interpolate_setpoints(self):
        # TODO(Isha) implement interpolation using JTC. Maybe from Gait class?
        return self.setpoints

    def to_joint_trajectory_point(self):
        # TODO(Isha) create trajectoryPoint msg here.
        pass

    def get_setpoint(self, index):
        return self.setpoints[index]

    def update_setpoint(self, index, setpoint):
        self.setpoints[index] = setpoint

    def within_safety_limits(self):
        for i in range(0, len(self.interpolatedSetpoints)):
            if self.interpolatedSetpoints[i] > self.limits.upper or self.interpolatedSetpoints[i] < self.limits.lower:
                return False
            if i > 0 and abs(self.interpolatedSetpoints[i] - self.interpolatedSetpoints[i - 1]) > self.limits.velocity:
                return False
            return True

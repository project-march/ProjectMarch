import numpy as np
from march_rqt_gait_generator.model.Limits import Limits
from march_rqt_gait_generator.model.Setpoint import Setpoint

import rospy
from scipy.interpolate import BPoly


class Joint:

    def __init__(self, name, limits, setpoints, duration):
        self.name = name
        self.limits = limits
        self.setpoints = self.enforce_position_limits(setpoints)
        self.duration = duration
        self.interpolatedSetpoints = self.interpolate_setpoints()

    def get_interpolated_value(self, time):
        for i in range(0, len(self.interpolatedSetpoints)):
            if self.interpolatedSetpoints[i] > time:
                return self.interpolatedSetpoints[i - 1]

    def interpolate_setpoints(self):
        # TODO(Isha) implement interpolation using JTC. Maybe from Gait class?

        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(0, len(time)):
            yi.append([position[i], velocity[i]])

        bpoly = BPoly.from_derivatives(time, yi)
        indices = np.linspace(0, self.duration, 100)
        return [indices, bpoly(indices)]

    def to_joint_trajectory_point(self):
        # TODO(Isha) create trajectoryPoint msg here.
        pass

    def get_setpoint(self, index):
        return self.setpoints[index]

    def set_setpoints(self, setpoints):
        self.setpoints = self.enforce_position_limits(setpoints)
        self.interpolatedSetpoints = self.interpolate_setpoints()

    def valid_setpoints(self, setpoints):
        for i in range(0, len(setpoints)):
            if setpoints[i].position > self.limits.upper or setpoints[i].position < self.limits.lower:
                return False
            if i > 0 and setpoints[i].time <= setpoints[i - 1].time:
                return False
        return True

    def enforce_position_limits(self, setpoints):
        for i in range(0, len(setpoints)):
            setpoints[i].position = min(max(setpoints[i].position, self.limits.lower), self.limits.upper)
        return setpoints

    def within_safety_limits(self):
        for i in range(0, len(self.interpolatedSetpoints)):
            if self.interpolatedSetpoints[i].position > self.limits.upper or \
                    self.interpolatedSetpoints[i].position < self.limits.lower:
                return False
            if i > 0 and abs(self.interpolatedSetpoints[i] - self.interpolatedSetpoints[i - 1]) > self.limits.velocity:
                return False
            return True

    def get_setpoints_unzipped(self):
        time = []
        position = []
        velocity = []

        for i in range(0, len(self.setpoints)):
            time.append(self.setpoints[i].time)
            position.append(self.setpoints[i].position)
            velocity.append(self.setpoints[i].velocity)

        return time, position, velocity

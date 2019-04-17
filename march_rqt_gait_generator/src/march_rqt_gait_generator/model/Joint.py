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
        self.interpolated_setpoints = self.interpolate_setpoints()

    def get_interpolated_position(self, time):
        for i in range(0, len(self.interpolated_setpoints[0])):
            if self.interpolated_setpoints[0][i] > time:
                return self.interpolated_setpoints[1][i - 1]

        return self.interpolated_setpoints[1][-1]

    def get_interpolated_setpoint(self, time):
        # If we have a setpoint this exact time there is no need to interpolate.
        for setpoint in self.setpoints:
            if setpoint.time == time:
                return setpoint

        interpolated_setpoints = self.interpolate_setpoints()
        for i in range(0, len(interpolated_setpoints[0])):
            if interpolated_setpoints[0][i] > time:
                position = interpolated_setpoints[1][i - 1]
                velocity = (interpolated_setpoints[1][i - 1] - interpolated_setpoints[1][i - 2])/(interpolated_setpoints[0][i-1]-interpolated_setpoints[0][i-2])
                return Setpoint(time, position, velocity)
        rospy.logerr("Could not interpolate setpoint at time " + str(time))
        return Setpoint(0, 0, 0)

    def interpolate_setpoints(self):
        # TODO(Isha) implement interpolation using JTC. Maybe from Gait class?

        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(0, len(time)):
            yi.append([position[i], velocity[i]])

        bpoly = BPoly.from_derivatives(time, yi)
        indices = np.linspace(0, self.duration, self.duration*1000)
        return [indices, bpoly(indices)]


    def get_setpoint(self, index):
        return self.setpoints[index]

    def set_setpoints(self, setpoints):
        self.setpoints = self.enforce_position_limits(setpoints)
        self.interpolated_setpoints = self.interpolate_setpoints()

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
        for i in range(0, len(self.interpolated_setpoints)):
            if self.interpolated_setpoints[i].position > self.limits.upper or \
                    self.interpolated_setpoints[i].position < self.limits.lower:
                return False
            if i > 0 and abs(self.interpolated_setpoints[i] - self.interpolated_setpoints[i - 1]) > self.limits.velocity:
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

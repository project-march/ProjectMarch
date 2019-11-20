import numpy as np
import copy
import rospy
from scipy.interpolate import BPoly
from numpy_ringbuffer import RingBuffer

from march_shared_classes.gait.joint_trajectory import JointTrajectory
from modifiable_setpoint import ModifiableSetpoint


class ModifiableJointTrajectory(JointTrajectory):

    def __init__(self, name, limits, setpoints, duration, gait_generator):
        self.name = name
        self.limits = limits
        self.setpoints = setpoints
        self.gait_generator = gait_generator
        self.setpoints_history = RingBuffer(capacity=100, dtype=list)
        self.setpoints_redo_list = RingBuffer(capacity=100, dtype=list)
        self.duration = duration

        self.enforce_limits()

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
                velocity = (interpolated_setpoints[1][i - 1] - interpolated_setpoints[1][i - 2]) \
                    / (interpolated_setpoints[0][i - 1] - interpolated_setpoints[0][i - 2])
                return ModifiableSetpoint(time, position, velocity)
        rospy.logerr("Could not interpolate setpoint at time " + str(time))
        return ModifiableSetpoint(0, 0, 0)

    def interpolate_setpoints(self):
        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(0, len(time)):
            yi.append([position[i], velocity[i]])

        bpoly = BPoly.from_derivatives(time, yi)
        indices = np.linspace(0, self.duration, self.duration * 100)
        return [indices, bpoly(indices)]

    def enforce_limits(self):
        for i in range(0, len(self.setpoints)):
            self.setpoints[i].set_position(min(max(self.setpoints[i].position, self.limits.lower), self.limits.upper))
            self.setpoints[i].set_velocity(min(
                max(self.setpoints[i].velocity, -self.limits.velocity),
                self.limits.velocity))

    def within_safety_limits(self):
        for i in range(0, len(self.interpolated_setpoints)):
            if self.interpolated_setpoints[i].position > self.limits.upper or \
                    self.interpolated_setpoints[i].position < self.limits.lower:
                return False
            if i > 0 and abs(
                    self.interpolated_setpoints[i] - self.interpolated_setpoints[i - 1]) > self.limits.velocity:
                return False
            return True

    def add_interpolated_setpoint(self, time):
        self.add_setpoint(self.get_interpolated_setpoint(time))

    def add_setpoint(self, setpoint):
        self.save_setpoints()
        # Calculate at what index the new setpoint should be added.
        new_index = len(self.setpoints)
        for i in range(0, len(self.setpoints)):
            if self.setpoints[i].time > setpoint.time:
                new_index = i
                break

        rospy.logdebug("adding setpoint " + str(setpoint) + " at index " + str(new_index))
        self.setpoints.insert(new_index, setpoint)

        self.enforce_limits()
        self.interpolated_setpoints = self.interpolate_setpoints()

    def remove_setpoint(self, index):
        self.save_setpoints()
        del self.setpoints[index]
        self.enforce_limits()
        self.interpolated_setpoints = self.interpolate_setpoints()

    def save_setpoints(self, single_joint_change=True):
        self.setpoints_history.append(copy.deepcopy(self.setpoints))    # list(...) to copy instead of pointer
        if single_joint_change:
            self.gait_generator.save_changed_joints([self])

    def invert(self):
        self.save_setpoints(single_joint_change=False)
        self.setpoints = list(reversed(self.setpoints))
        for setpoint in self.setpoints:
            setpoint.invert(self.duration)
        self.interpolated_setpoints = self.interpolate_setpoints()

    def undo(self):
        self.setpoints_redo_list.append(list(self.setpoints))
        self.setpoints = self.setpoints_history.pop()
        self.interpolated_setpoints = self.interpolate_setpoints()

    def redo(self):
        self.setpoints_history.append(list(self.setpoints))
        self.setpoints = self.setpoints_redo_list.pop()
        self.interpolated_setpoints = self.interpolate_setpoints()

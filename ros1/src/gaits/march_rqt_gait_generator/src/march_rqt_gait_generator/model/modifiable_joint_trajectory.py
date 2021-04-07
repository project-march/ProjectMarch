import copy

from numpy_ringbuffer import RingBuffer
import rospy

from march_shared_classes.gait.joint_trajectory import JointTrajectory

from .modifiable_setpoint import ModifiableSetpoint


class ModifiableJointTrajectory(JointTrajectory):
    setpoint_class = ModifiableSetpoint

    def __init__(self, name, limits, setpoints, duration, gait_generator=None):
        super(ModifiableJointTrajectory, self).__init__(
            name, limits, setpoints, duration
        )

        self.setpoints_history = RingBuffer(capacity=100, dtype=list)
        self.setpoints_redo_list = RingBuffer(capacity=100, dtype=list)
        self.gait_generator = gait_generator

        self._start_point = None
        self._end_point = None

        if self.setpoints[0].time != 0:
            rospy.logwarn(
                "First setpoint of {0} has been set "
                "from {1} to 0".format(name, self.setpoints[0].time)
            )
        if self.setpoints[-1].time != duration:
            rospy.logwarn(
                "Last setpoint of {0} has been set "
                "from {1} to {2}".format(name, self.setpoints[0].time, duration)
            )

    def set_setpoints(self, setpoints):
        self.setpoints = setpoints

    def set_duration(self, new_duration, rescale=True):
        super(ModifiableJointTrajectory, self).set_duration(new_duration, rescale)
        self.enforce_limits()

    @JointTrajectory.setpoints.setter
    def setpoints(self, setpoints):
        self._setpoints = setpoints
        self.enforce_limits()
        self.interpolate_setpoints()

    def enforce_limits(self):
        if self.start_point:
            self.setpoints[0].position = self.start_point.position
            self.setpoints[0].velocity = self.start_point.velocity
        if self.end_point:
            self.setpoints[-1].position = self.end_point.position
            self.setpoints[-1].velocity = self.end_point.velocity

        self.setpoints[0].time = 0
        self.setpoints[-1].time = self.duration

        for setpoint in self.setpoints:
            setpoint.position = min(
                max(setpoint.position, self.limits.lower), self.limits.upper
            )
            setpoint.velocity = min(
                max(setpoint.velocity, -self.limits.velocity), self.limits.velocity
            )

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

        self.setpoints.insert(new_index, setpoint)
        self.enforce_limits()
        self.interpolate_setpoints()

    def replace_setpoint(self, index, new_setpoint):
        self.save_setpoints()
        self.setpoints[index] = new_setpoint
        self.enforce_limits()
        self.interpolate_setpoints()

    def remove_setpoint(self, index):
        self.save_setpoints()
        del self.setpoints[index]
        self.enforce_limits()
        self.interpolate_setpoints()

    def save_setpoints(self, single_joint_change=True):
        self.setpoints_history.append(
            {
                "setpoints": copy.deepcopy(self.setpoints),
                "start_point": self.start_point,
                "end_point": self.end_point,
            }
        )
        if single_joint_change:
            self.gait_generator.save_changed_settings({"joints": [self]})

    def invert(self):
        self.save_setpoints(single_joint_change=False)
        for setpoint in self.setpoints:
            setpoint.invert(self.duration)
        self.setpoints = list(reversed(self.setpoints))
        self.interpolate_setpoints()

    def undo(self):
        if not self.setpoints_history:
            return

        self.setpoints_redo_list.append(
            {
                "setpoints": copy.deepcopy(self.setpoints),
                "start_point": self.start_point,
                "end_point": self.end_point,
            }
        )
        setpoints = self.setpoints_history.pop()
        self._setpoints = setpoints["setpoints"]
        self._start_point = setpoints["start_point"]
        self._end_point = setpoints["end_point"]
        self._duration = self.setpoints[-1].time
        self.enforce_limits()
        self.interpolate_setpoints()

    def redo(self):
        if not self.setpoints_redo_list:
            return

        self.setpoints_history.append(
            {
                "setpoints": copy.deepcopy(self.setpoints),
                "start_point": self.start_point,
                "end_point": self.end_point,
            }
        )
        setpoints = self.setpoints_redo_list.pop()
        self._setpoints = setpoints["setpoints"]
        self._start_point = setpoints["start_point"]
        self._end_point = setpoints["end_point"]
        self._duration = self.setpoints[-1].time
        self.enforce_limits()
        self.interpolate_setpoints()

    @property
    def start_point(self):
        return self._start_point

    @start_point.setter
    def start_point(self, start_point):
        self._start_point = start_point
        self.enforce_limits()
        self.interpolated_setpoints = self.interpolate_setpoints()

    @property
    def end_point(self):
        return self._end_point

    @end_point.setter
    def end_point(self, end_point):
        self._end_point = end_point
        self.enforce_limits()
        self.interpolated_setpoints = self.interpolate_setpoints()

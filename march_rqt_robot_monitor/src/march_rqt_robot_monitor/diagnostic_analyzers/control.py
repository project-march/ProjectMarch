
from diagnostic_msgs.msg import DiagnosticStatus
import rospy
from urdf_parser_py import urdf


class CheckJointValues(object):
    """Base class to diagnose the joint movement values."""

    def __init__(self, topic, msg_type):
        rospy.Subscriber(topic, msg_type, self.cb)

        # callback variables
        self._timestamp = None
        self._joint_names = None
        self._position = None
        self._velocity = None
        self._effort = None

        # robot properties
        self._lower_soft_limits = {}
        self._upper_soft_limits = {}
        self._velocity_limits = {}
        self._effort_limits = {}

        self._robot = urdf.Robot.from_parameter_server('/robot_description')
        for joint in self._robot.joints:
            try:
                self._lower_soft_limits[joint.name] = joint.safety_controller.soft_lower_limit
                self._upper_soft_limits[joint.name] = joint.safety_controller.soft_upper_limit
                self._velocity_limits[joint.name] = joint.limit.velocity
                self._effort_limits[joint.name] = joint.limit.effort
            except AttributeError:
                pass

    def cb(self, msg):
        """Save the latest published movement values with corresponding timestamp."""
        self._timestamp = msg.header.stamp
        self._joint_names = msg.name
        self._position = msg.position
        self._velocity = msg.velocity
        self._effort = msg.effort

    def position_diagnostics(self, stat):
        """The diagnostic message to display the positions in standard format."""
        if self._timestamp is None:
            stat.add(' Topic error ', 'No events recorded.')
            return stat

        stat.add('Timestamp', self._timestamp)

        joint_outside_soft_limits = []
        for index in range(len(self._joint_names)):
            stat.add(self._joint_names[index], self._position[index])

            if self.is_within_limit(self._position[index], self._joint_names[index], self._upper_soft_limits):
                joint_outside_soft_limits.append(self._joint_names[index])

            if self.is_within_limit(self._position[index], self._joint_names[index], self._lower_soft_limits, 'lower'):
                joint_outside_soft_limits.append(self._joint_names[index])

        if joint_outside_soft_limits:
            stat.summary(DiagnosticStatus.ERROR, 'Outside soft limits: {ls}'.format(ls=str(joint_outside_soft_limits)))
        else:
            stat.summary(DiagnosticStatus.OK, 'OK')

    def velocity_diagnostics(self, stat):
        """The diagnostic message to display the velocities in standard format."""
        if self._timestamp is None:
            stat.add(' Topic error ', 'No events recorded.')
            return stat

        stat.summary(DiagnosticStatus.OK, 'OK')
        stat.add('Timestamp', self._timestamp)

        joints_at_velocity_limit = []
        for index in range(len(self._joint_names)):
            stat.add(self._joint_names[index], self._velocity[index])

            if self.is_within_limit(self._velocity[index], self._joint_names[index], self._velocity_limits):
                joints_at_velocity_limit.append(self._joint_names[index])

        if joints_at_velocity_limit:
            stat.summary(DiagnosticStatus.ERROR, 'At velocity limit: {ls}'.format(ls=str(joints_at_velocity_limit)))
        else:
            stat.summary(DiagnosticStatus.OK, 'OK')

    def effort_diagnostics(self, stat):
        """The diagnostic message to display the efforts in standard format."""
        if self._timestamp is None:
            stat.add(' Topic error ', 'No events recorded.')
            return stat

        stat.summary(DiagnosticStatus.OK, 'OK')
        stat.add('Timestamp', self._timestamp)

        joints_at_effort_limit = []
        for index in range(len(self._joint_names)):
            stat.add(self._joint_names[index], self._position[index])

            if self.is_within_limit(self._effort[index], self._joint_names[index], self._effort_limits):
                joints_at_effort_limit.append(self._joint_names[index])

        if joints_at_effort_limit:
            stat.summary(DiagnosticStatus.ERROR, 'At effort limit: {ls}'.format(ls=str(joints_at_effort_limit)))
        else:
            stat.summary(DiagnosticStatus.OK, 'OK')

    @staticmethod
    def is_within_limit(value, joint_name, limit_dict, operator='upper'):
        if operator == 'upper':
            if value >= limit_dict[joint_name]:
                return True
        elif operator == 'lower':
            if value <= limit_dict[joint_name]:
                return True
        else:
            raise KeyError('Non valid operator given')

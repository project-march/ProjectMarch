
from diagnostic_msgs.msg import DiagnosticStatus
import rospy


class CheckJointValues(object):
    """Base class to diagnose the joint movement values."""

    def __init__(self, topic, msg_type):
        rospy.Subscriber(topic, msg_type, self.cb)

        self._timestamp = None
        self._joint_names = None
        self._position = None
        self._velocity = None
        self._effort = None

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

        stat.summary(DiagnosticStatus.OK, 'OK')
        stat.add('Timestamp', self._timestamp)

        for index in range(len(self._joint_names)):
            stat.add(self._joint_names[index], self._position[index])

    def velocity_diagnostics(self, stat):
        """The diagnostic message to display the velocities in standard format."""
        if self._timestamp is None:
            stat.add(' Topic error ', 'No events recorded.')
            return stat

        stat.summary(DiagnosticStatus.OK, 'OK')
        stat.add('Timestamp', self._timestamp)

        for index in range(len(self._joint_names)):
            stat.add(self._joint_names[index], self._position[index])

    def effort_diagnostics(self, stat):
        """The diagnostic message to display the efforts in standard format."""
        if self._timestamp is None:
            stat.add(' Topic error ', 'No events recorded.')
            return stat

        stat.summary(DiagnosticStatus.OK, 'OK')
        stat.add('Timestamp', self._timestamp)

        for index in range(len(self._joint_names)):
            stat.add(self._joint_names[index], self._position[index])


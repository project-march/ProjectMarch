
from diagnostic_msgs.msg import DiagnosticStatus
import rospy


LOWER_THRESHOLD_VALID_TEMPERATURE_VALUE = 0
UPPER_THRESHOLD_VALID_TEMPERATURE_VALUE = 100


class CheckJointTemperature(object):
    """Base class to diagnose the joint temperatures."""

    def __init__(self, joint_name, topic, msg_type):
        rospy.Subscriber(topic, msg_type, self.cb)

        self._default_thresholds = \
            rospy.get_param('safety_node/default_temperature_threshold', None)
        self._thresholds_warning = \
            rospy.get_param('safety_node/temperature_thresholds_warning/{jn}'.format(jn=joint_name), None)
        self._thresholds_non_fatal = \
            rospy.get_param('safety_node/temperature_thresholds_non_fatal/{jn}'.format(jn=joint_name), None)
        self._thresholds_fatal = \
            rospy.get_param('safety_node/temperature_thresholds_fatal/{jn}'.format(jn=joint_name), None)

        self._timestamp = None
        self._temperature = None

    def cb(self, msg):
        """Save the latest published temperature with corresponding timestamp."""
        self._temperature = float(msg.temperature)
        self._timestamp = msg.header.stamp

    def diagnostics(self, stat):
        """The diagnostic message to display in the standard stat format."""
        if self._timestamp is None:
            stat.add(' Topic error ', 'No events recorded.')
            return stat

        if not LOWER_THRESHOLD_VALID_TEMPERATURE_VALUE < self._temperature < UPPER_THRESHOLD_VALID_TEMPERATURE_VALUE:
            stat.add('Current temperature', self._temperature)
            stat.summary(DiagnosticStatus.WARN, 'No valid temperature value ({tp}).'.format(tp=self._temperature))
            return stat

        stat.add('Default threshold', self._default_thresholds)
        stat.add('Warning threshold', self._thresholds_warning)
        stat.add('Non fatal threshold', self._thresholds_non_fatal)
        stat.add('Fatal threshold', self._thresholds_fatal)
        stat.add('Current temperature', self._temperature)

        if self._temperature < self._thresholds_warning:
            stat.summary(DiagnosticStatus.OK, 'OK ({tp}).'.format(tp=self._temperature))
        elif self._thresholds_warning < self._temperature < self._thresholds_non_fatal:
            stat.summary(DiagnosticStatus.WARN, 'Temperature almost too high ({tp}).'.format(tp=self._temperature))
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Temperature to high ({tp}).'.format(tp=self._temperature))

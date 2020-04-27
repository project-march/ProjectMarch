
from diagnostic_msgs.msg import DiagnosticStatus
import rospy
from sensor_msgs.msg import Temperature


LOWER_THRESHOLD_VALID_VALUE = 0
UPPER_THRESHOLD_VALID_VALUE = 100


class CheckJointTemperature(object):
    """Base class to diagnose the joint temperatures."""

    def __init__(self, joint_name, topic):
        rospy.Subscriber(topic, Temperature, self.cb)

        self._default_thresholds = \
            rospy.get_param('/safety_node/default_temperature_threshold', None)
        self._thresholds_warning = \
            rospy.get_param('/safety_node/temperature_thresholds_warning/{jn}'.format(jn=joint_name), None)
        self._thresholds_non_fatal = \
            rospy.get_param('/safety_node/temperature_thresholds_non_fatal/{jn}'.format(jn=joint_name), None)
        self._thresholds_fatal = \
            rospy.get_param('/safety_node/temperature_thresholds_fatal/{jn}'.format(jn=joint_name), None)

        self._timestamp = None
        self._temperature = None

    def cb(self, message):
        """Save the latest published temperature with corresponding timestamp."""
        self._temperature = float(message.temperature)
        self._timestamp = message.header.stamp

    def diagnostics(self, stat):
        """The diagnostic message to display in the standard stat format."""
        if self._timestamp is None:
            stat.add(' Topic error ', 'No events recorded.')
            return stat

        if self._temperature < LOWER_THRESHOLD_VALID_VALUE or self._temperature < UPPER_THRESHOLD_VALID_VALUE:
            stat.summary(DiagnosticStatus.WARN, 'No valid temperature value.')
            return stat

        stat.add('Default threshold', self._default_thresholds)
        stat.add('Warning threshold', self._thresholds_warning)
        stat.add('Non fatal threshold', self._thresholds_non_fatal)
        stat.add('Fatal threshold', self._thresholds_fatal)
        stat.add('Current temperature', self._temperature)

        if self._temperature < self._thresholds_warning:
            stat.summary(DiagnosticStatus.OK, 'OK')
        elif self._thresholds_warning < self._temperature < self._thresholds_non_fatal:
            stat.summary(DiagnosticStatus.WARN, 'Temperature in warning zone.')
        else:
            stat.summary(DiagnosticStatus.ERROR, 'Temperature to high.')


def temperature_updater(general_updater):
    """Add all the temperature checks to the general updater.

    :param general_updater: the general diagnostic updater (diagnostic_updater.Updater)
    """
    check_temp_joint_left_ankle = CheckJointTemperature('Temperature left ankle', '/march/temperature/left_ankle')
    general_updater.add('Temperature left ankle', check_temp_joint_left_ankle.diagnostics)

    check_temp_joint_left_knee = CheckJointTemperature('Temperature left knee', '/march/temperature/left_knee')
    general_updater.add('Temperature left knee', check_temp_joint_left_knee.diagnostics)

    check_temp_joint_left_hip_fe = CheckJointTemperature('Temperature left hip FE', '/march/temperature/left_hip_fe')
    general_updater.add('Temperature left hip FE', check_temp_joint_left_hip_fe.diagnostics)

    check_temp_joint_left_hip_aa = CheckJointTemperature('Temperature left hip AA', '/march/temperature/left_hip_aa')
    general_updater.add('Temperature left hip AA', check_temp_joint_left_hip_aa.diagnostics)

    check_temp_joint_right_ankle = CheckJointTemperature('Temperature right ankle', '/march/temperature/right_ankle')
    general_updater.add('Temperature right ankle', check_temp_joint_right_ankle.diagnostics)

    check_temp_joint_right_knee = CheckJointTemperature('Temperature right knee', '/march/temperature/right_knee')
    general_updater.add('Temperature right knee', check_temp_joint_right_knee.diagnostics)

    check_temp_joint_right_hip_fe = CheckJointTemperature('Temperature right hip FE', '/march/temperature/right_hip_fe')
    general_updater.add('Temperature right hip FE', check_temp_joint_right_hip_fe.diagnostics)

    check_temp_joint_right_hip_aa = CheckJointTemperature('Temperature right hip AA', '/march/temperature/right_hip_aa')
    general_updater.add('Temperature right hip AA', check_temp_joint_right_hip_aa.diagnostics)

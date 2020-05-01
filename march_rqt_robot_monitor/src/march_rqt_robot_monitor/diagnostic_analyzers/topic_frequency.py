
import diagnostic_updater
import rospy


class CheckTopicFrequency(object):
    """Base class to diagnose whether the IPD is connected properly."""

    def __init__(self, name, topic, message_type, general_updater, frequency):
        self._name = name
        self._frequency_bounds = {'min': frequency, 'max': frequency}

        self._topic_frequency = diagnostic_updater.HeaderlessTopicDiagnostic(
            topic, general_updater, diagnostic_updater.FrequencyStatusParam(self._frequency_bounds, 0.1, 10))
        rospy.Subscriber(topic, message_type, self.cb)

    def cb(self, msg):
        """Update the frequency checker."""
        self._topic_frequency.tick()

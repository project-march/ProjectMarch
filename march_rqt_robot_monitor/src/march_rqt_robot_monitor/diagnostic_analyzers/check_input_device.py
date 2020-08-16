from diagnostic_updater import FrequencyStatusParam, HeaderlessTopicDiagnostic
import rospy


class CheckInputDevice(object):
    """Base class to diagnose whether the input devices are connected properly."""

    def __init__(self, topic, message_type, updater, frequency):
        self._frequency_params = FrequencyStatusParam({'min': frequency}, 0.1)
        self._updater = updater
        self._diagnostics = {}

        rospy.Subscriber(topic, message_type, self._cb)

    def _cb(self, msg):
        """
        Update the frequency diagnostics for given input device.

        :type msg: march_shared_resources.msg.Alive
        :param msg: Alive message
        """
        if msg.id in self._diagnostics:
            self._diagnostics[msg.id].tick()
        else:
            self._diagnostics[msg.id] = HeaderlessTopicDiagnostic('input_device {0}'.format(msg.id),
                                                                  self._updater, self._frequency_params)
            self._updater.force_update()

"""The module check_input_device.py contains the CheckInputDevice Class."""

from diagnostic_updater import FrequencyStatusParam, HeaderlessTopicDiagnostic, Updater
from rclpy.node import Node

from march_shared_msgs.msg import Alive


class CheckInputDevice:
    """Base class to diagnose whether the input devices are connected properly."""

    def __init__(
        self,
        node: Node,
        topic: str,
        message_type: type,
        updater: Updater,
        frequency: float,
    ):
        self._frequency_params = FrequencyStatusParam({"min": frequency})
        self._updater = updater
        self._diagnostics = {}

        node.create_subscription(message_type, topic, self.alive_callback, qos_profile=10)

    def alive_callback(self, msg: Alive) -> None:
        """Callback function that updates the frequency diagnostics for given input device.

        Args:
            msg (march_shared_msgs.msg.Alive): Alive message.
        """
        if msg.id in self._diagnostics:
            self._diagnostics[msg.id].tick()
        else:
            self._diagnostics[msg.id] = HeaderlessTopicDiagnostic(
                f"input_device {msg.id}", self._updater, self._frequency_params
            )

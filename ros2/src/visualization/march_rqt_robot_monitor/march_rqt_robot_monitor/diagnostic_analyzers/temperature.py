"""The module temperature.py contains the CheckJointTemperature Class."""

from typing import List

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper
from march_utility.utilities.node_utils import wait_for_service
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from sensor_msgs.msg import Temperature

LOWER_THRESHOLD_VALID_TEMPERATURE_VALUE = 0
UPPER_THRESHOLD_VALID_TEMPERATURE_VALUE = 100

THRESHOLD_TYPES = ["warning", "non_fatal", "fatal"]


class CheckJointTemperature:
    """Base class to diagnose the joint temperatures."""

    def __init__(self, node: Node, joint_name: str, topic: str, msg_type: type):
        self.node = node
        self.joint_name = joint_name

        node.create_subscription(msg_type, topic, self._cb, qos_profile=10)

        (
            self._default_threshold,
            self._thresholds_warning,
            self._thresholds_non_fatal,
            self._thresholds_fatal,
        ) = self._get_temperature_thresholds()

        self._timestamp = None
        self._temperature = None

    def _get_temperature_thresholds(self) -> List[float]:
        """Get the temperature thresholds for this joint.

        :return Returns a tuple containing all temperature thresholds
        """
        names = ["/march/safety_node/default_temperature_threshold"] + [
            f"march/safety_node/temperature_thresholds_{threshold_type}/{self.joint_name}"
            for threshold_type in THRESHOLD_TYPES
        ]
        client = self.node.create_client(
            srv_type=GetParameters, srv_name="/march/safety_node/get_parameters"
        )
        wait_for_service(self.node, client)
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self.node, future)

        return [value.double_value for value in future.result().value]

    def _cb(self, msg: Temperature):
        """Save the latest published temperature with corresponding timestamp."""
        self._temperature = float(msg.temperature)
        self._timestamp = msg.header.stamp

    def diagnostics(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        """Create a diagnostic message to display in the standard stat format."""
        if self._timestamp is None:
            stat.add("Topic error", "No events recorded")
            stat.summary(DiagnosticStatus.STALE, "No temperature recorded")
            return stat

        if (
            not LOWER_THRESHOLD_VALID_TEMPERATURE_VALUE
            < self._temperature
            < UPPER_THRESHOLD_VALID_TEMPERATURE_VALUE
        ):
            stat.add("Current temperature", self._temperature)
            stat.summary(
                DiagnosticStatus.WARN,
                "No valid temperature value ({tp}).".format(tp=self._temperature),
            )
            return stat

        stat.add("Default threshold", self._default_threshold)
        stat.add("Warning threshold", self._thresholds_warning)
        stat.add("Non fatal threshold", self._thresholds_non_fatal)
        stat.add("Fatal threshold", self._thresholds_fatal)
        stat.add("Current temperature", self._temperature)

        if self._default_threshold is None:
            stat.summary(DiagnosticStatus.WARN, "No thresholds found")
        elif self._temperature < self._thresholds_warning:
            stat.summary(DiagnosticStatus.OK, "OK ({tp}).".format(tp=self._temperature))
        elif self._temperature < self._thresholds_non_fatal:
            stat.summary(
                DiagnosticStatus.WARN,
                "Temperature almost too high ({tp}).".format(tp=self._temperature),
            )
        else:
            stat.summary(
                DiagnosticStatus.ERROR,
                "Temperature too high ({tp}).".format(tp=self._temperature),
            )

        return stat

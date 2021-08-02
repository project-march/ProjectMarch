"""The module motor_controller_state.py contains the CheckMotorControllerStatus Class."""

from typing import List, Callable

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from rclpy.node import Node

from march_shared_msgs.msg import PowerDistributionBoardData


class CheckBatteryStatus:
    """Base class to diagnose the pdb battery statuses."""

    BATTERY_PERCENTAGE_WARNING_THRESHOLD = 20
    BATTERY_PERCENTAGE_ERROR_THRESHOLD = 5
    BATTERY_VOLTAGE_WARNING_THRESHOLD = 48
    BATTERY_VOLTAGE_ERROR_THRESHOLD = 46
    BATTERY_TEMPERATURE_WARNING_THRESHOLD = 40
    BATTERY_TEMPERATURE_ERROR_THRESHOLD = 50

    def __init__(self, node: Node, updater: Updater):
        """Initialize a PDB diagnostic which analyzes battery percentage, voltage
        and temperature.

        :type updater: diagnostic_updater.Updater
        """
        self.node = node
        self._sub = node.create_subscription(
            msg_type=PowerDistributionBoardData,
            topic="/march/pdb_data",
            callback=self._cb,
            qos_profile=10,
        )
        self._pdb_data = None

        updater.add(f"Battery", self._diagnostic())

    def _cb(self, msg: PowerDistributionBoardData):
        """Set the motor_controller_states.

        :type msg: MotorControllerState
        """
        self._pdb_data = msg

    def _diagnostic(self) -> Callable:  # noqa: D202
        """Create a diagnostic function for an MotorController.

        :type index: int
        :param index: index of the joint

        :return Curried diagnostic function that updates the diagnostic status
                according to the given index.
        """

        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No battert")
                return stat

            battery_percentage = self._pdb_data.battery_state.percentage
            battery_voltage = self._pdb_data.battery_state.voltage
            battery_temperature = self._pdb_data.battery_state.temperature

            stat.add("Battery percentage", battery_percentage)
            stat.add("Battery voltage", battery_voltage)
            stat.add("Battery temperature", battery_temperature)

            if battery_percentage <= self.BATTERY_PERCENTAGE_ERROR_THRESHOLD:
                stat.summary(DiagnosticStatus.ERROR, battery_percentage)
            elif battery_voltage <= self.BATTERY_VOLTAGE_ERROR_THRESHOLD:
                stat.summary(DiagnosticStatus.ERROR, battery_voltage)
            elif battery_temperature >= self.BATTERY_TEMPERATURE_ERROR_THRESHOLD:
                stat.summary(DiagnosticStatus.ERROR, battery_temperature)
            elif battery_percentage <= self.BATTERY_PERCENTAGE_WARNING_THRESHOLD:
                stat.summary(DiagnosticStatus.WARN, battery_percentage)
            elif battery_voltage <= self.BATTERY_VOLTAGE_WARNING_THRESHOLD:
                stat.summary(DiagnosticStatus.WARN, battery_voltage)
            elif battery_temperature >= self.BATTERY_TEMPERATURE_WARNING_THRESHOLD:
                stat.summary(DiagnosticStatus.WARN, battery_temperature)
            else:
                stat.summary(DiagnosticStatus.OK, "Battery all good")

            return stat

        return d

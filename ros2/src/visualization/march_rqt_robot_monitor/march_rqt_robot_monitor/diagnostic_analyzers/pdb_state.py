"""The module motor_controller_state.py contains the CheckMotorControllerStatus Class."""

from typing import List, Callable

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from rclpy.node import Node

from march_shared_msgs.msg import PowerDistributionBoardData


class CheckPDBStatus:
    """Base class to diagnose the motor_controller statuses."""

    BATTERY_PERCENTAGE_WARNING_THRESHOLD = 20
    BATTERY_PERCENTAGE_ERROR_THRESHOLD = 5
    BATTERY_VOLTAGE_WARNING_THRESHOLD = 48
    BATTERY_VOLTAGE_ERROR_THRESHOLD = 46
    BATTERY_TEMPERATURE_WARNING_THRESHOLD = 40
    BATTERY_TEMPERATURE_ERROR_THRESHOLD = 50

    def __init__(self, node: Node, updater: Updater):
        """Initialize an PDB diagnostic which analyzes MotorController states.

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

        updater.add(f"PowerDistributionBoard stop state", self._stop_diagnostic())
        updater.add(f"PowerDistributionBoard lv", self._lv_diagnostic())
        updater.add(f"PowerDistributionBoard hv", self._hv_diagnostic())
        updater.add(f"Battery percentage", self._battery_percentage_diagnostic())
        updater.add(f"Battery voltage", self._battery_voltage_diagnostic())
        updater.add(f"Battery temperature", self._battery_temperature_diagnostic())

    def _cb(self, msg: PowerDistributionBoardData):
        """Set the motor_controller_states.

        :type msg: MotorControllerState
        """
        self._pdb_data = msg

    def _battery_temperature_diagnostic(self) -> Callable:  # noqa: D202
        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No battery data")
                return stat

            battery_temperature = self._pdb_data.battery_state.temperature
            if battery_temperature >= self.BATTERY_TEMPERATURE_ERROR_THRESHOLD:
                stat.summary(
                    DiagnosticStatus.ERROR,
                    f"Battery temperature too high: " f"{battery_temperature}",
                )
            elif battery_temperature >= self.BATTERY_TEMPERATURE_WARNING_THRESHOLD:
                stat.summary(
                    DiagnosticStatus.WARN,
                    f"Battery temperature high: " f"{battery_temperature}",
                )
            else:
                stat.summary(DiagnosticStatus.OK, f"OK: {battery_temperature}")
            return stat

        return d

    def _battery_percentage_diagnostic(self) -> Callable:  # noqa: D202
        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No battery data")
                return stat

            battery_percentage = self._pdb_data.battery_state.percentage
            if battery_percentage <= self.BATTERY_PERCENTAGE_ERROR_THRESHOLD:
                stat.summary(
                    DiagnosticStatus.ERROR,
                    f"Battery percentage too low: " f"{battery_percentage}",
                )
            elif battery_percentage <= self.BATTERY_PERCENTAGE_WARNING_THRESHOLD:
                stat.summary(
                    DiagnosticStatus.WARN,
                    f"Battery percentage low: " f"{battery_percentage}",
                )
            else:
                stat.summary(DiagnosticStatus.OK, f"OK: {battery_percentage}")
            return stat

        return d

    def _battery_voltage_diagnostic(self) -> Callable:  # noqa: D202:
        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No battery data")
                return stat

            battery_voltage = self._pdb_data.battery_state.voltage
            if battery_voltage <= self.BATTERY_VOLTAGE_ERROR_THRESHOLD:
                stat.summary(
                    DiagnosticStatus.ERROR,
                    f"Battery voltage too low: " f"{battery_voltage}",
                )
            elif battery_voltage <= self.BATTERY_PERCENTAGE_WARNING_THRESHOLD:
                stat.summary(
                    DiagnosticStatus.WARN, f"Battery voltage low: " f"{battery_voltage}"
                )
            else:
                stat.summary(DiagnosticStatus.OK, f"OK: {battery_voltage}")

            return stat

        return d

    def _lv_diagnostic(self) -> Callable:  # noqa: D202
        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No pdb data")
                return stat

            lv1_state = self._pdb_data.lv_state.lv1_ok
            lv2_state = self._pdb_data.lv_state.lv2_ok

            if lv1_state != 1 or lv2_state != 1:
                stat.summary(
                    DiagnosticStatus.ERROR,
                    f"LV error,\n LV1 state:"
                    f" {lv1_state}\n "
                    f"LV2 state: {lv2_state}",
                )

            else:
                stat.summary(
                    DiagnosticStatus.OK,
                    f"LV OK\n"
                    f"Current lv1: "
                    f"{self._pdb_data.lv_state.lv1_current}\n"
                    f"Current lv2: "
                    f"{self._pdb_data.lv_state.lv2_current}",
                )
            return stat

        return d

    def _hv_diagnostic(self) -> Callable:  # noqa: D202
        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No pdb data")
                return stat

            hv_state = self._pdb_data.hv_state

            stat.summary(
                DiagnosticStatus.OK,
                f"HV OK\n"
                f"Total current: "
                f"{hv_state.total_current}\n"
                f"Current hv1: "
                f"{hv_state.hv1_current}\n"
                f"Current hv2: "
                f"{hv_state.hv2_current}\n"
                f"Current hv3: "
                f"{hv_state.hv3_current}\n"
                f"Current hv4: "
                f"{hv_state.hv4_current}",
            )
            return stat

        return d

    def _stop_diagnostic(self) -> Callable:  # noqa: D202
        """Create a diagnostic function for an MotorController.

        :type index: int
        :param index: index of the joint

        :return Curried diagnostic function that updates the diagnostic status
                according to the given index.
        """

        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No pdb data")
                return stat

            emergency_button_status = self._pdb_data.emergency_button_state
            stop_button_state = self._pdb_data.stop_button_state

            stat.add("Emergency button status", str(emergency_button_status))
            stat.add("Stop button status", str(stop_button_state))

            if emergency_button_status != 1:
                stat.summary(DiagnosticStatus.ERROR, f"Emergency button pressed")
            elif stop_button_state != 1:
                stat.summary(DiagnosticStatus.ERROR, f"Stop button pressed")
            else:
                stat.summary(
                    DiagnosticStatus.OK, "OK, no stop or emergency button " "pressed"
                )

            return stat

        return d

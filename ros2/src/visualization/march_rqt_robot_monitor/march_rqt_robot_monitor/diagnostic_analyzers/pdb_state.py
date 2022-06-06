"""The module pdb_state.py contains the CheckPDBStatus Class."""

from typing import Callable

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from rclpy.node import Node

from march_shared_msgs.msg import PowerDistributionBoardData


class CheckPDBStatus:
    """Base class to diagnose the PDB statuses."""

    BATTERY_PERCENTAGE_WARNING_THRESHOLD = 30
    BATTERY_PERCENTAGE_ERROR_THRESHOLD = 10
    BATTERY_VOLTAGE_WARNING_THRESHOLD = 48
    BATTERY_VOLTAGE_ERROR_THRESHOLD = 47.5
    BATTERY_TEMPERATURE_WARNING_THRESHOLD = 40
    BATTERY_TEMPERATURE_ERROR_THRESHOLD = 50

    def __init__(self, node: Node, updater: Updater):
        """Initialize an PDB diagnostic which analyzes PDB states.

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

        updater.add(
            "PowerDistributionBoard stop state",
            self._pdb_diagnostic(self._stop_diagnostic),
        )
        updater.add("PowerDistributionBoard lv", self._pdb_diagnostic(self._lv_diagnostic))
        updater.add("PowerDistributionBoard hv", self._pdb_diagnostic(self._hv_diagnostic))
        updater.add("Battery percentage", self._pdb_diagnostic(self._battery_percentage_check))
        updater.add("Battery voltage", self._pdb_diagnostic(self._battery_voltage_check))
        updater.add("Battery temperature", self._pdb_diagnostic(self._battery_temperature_check))

    def _cb(self, msg: PowerDistributionBoardData):
        """Set the pdb state.

        :type msg: PowerDistributionBoardData
        """
        self._pdb_data = msg

    @staticmethod
    def _threshold_check(
        stat: DiagnosticStatusWrapper,
        value: float,
        warning_threshold: float,
        error_threshold: float,
        is_upper_limit: bool,
        value_name: str,
    ):
        if value == -1:
            stat.summary(DiagnosticStatus.STALE, "No PDB data")
            return stat

        if (is_upper_limit and value >= warning_threshold) or (not is_upper_limit and value <= warning_threshold):
            stat.summary(
                DiagnosticStatus.WARN,
                f"{value_name} {'high' if is_upper_limit else 'low'}: {value}",
            )
        elif (is_upper_limit and value >= error_threshold) or (not is_upper_limit and value <= error_threshold):
            stat.summary(
                DiagnosticStatus.WARN,
                f"{value_name} too {'high' if is_upper_limit else 'low'}: {value}",
            )
        else:
            stat.summary(DiagnosticStatus.OK, f"OK: {value}")

        return stat

    def _pdb_diagnostic(self, check_function) -> Callable:  # noqa: D202
        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._pdb_data is None:
                stat.summary(DiagnosticStatus.STALE, "No PDB data")
                return stat

            check_function(stat)

            return stat

        return d

    def _battery_temperature_check(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        battery_temperature = self._pdb_data.battery_state.temperature

        return CheckPDBStatus._threshold_check(
            stat=stat,
            value=battery_temperature,
            warning_threshold=self.BATTERY_TEMPERATURE_WARNING_THRESHOLD,
            error_threshold=self.BATTERY_VOLTAGE_ERROR_THRESHOLD,
            is_upper_limit=True,
            value_name="Battery temperature",
        )

    def _battery_percentage_check(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        battery_percentage = self._pdb_data.battery_state.percentage

        return CheckPDBStatus._threshold_check(
            stat=stat,
            value=battery_percentage,
            warning_threshold=self.BATTERY_PERCENTAGE_WARNING_THRESHOLD,
            error_threshold=self.BATTERY_PERCENTAGE_ERROR_THRESHOLD,
            is_upper_limit=False,
            value_name="Battery percentage",
        )

    def _battery_voltage_check(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        battery_voltage = self._pdb_data.battery_state.voltage

        return CheckPDBStatus._threshold_check(
            stat=stat,
            value=battery_voltage,
            warning_threshold=self.BATTERY_VOLTAGE_WARNING_THRESHOLD,
            error_threshold=self.BATTERY_VOLTAGE_ERROR_THRESHOLD,
            is_upper_limit=False,
            value_name="Battery voltage",
        )

    def _lv_diagnostic(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        stat.summary(
            DiagnosticStatus.OK,
            f"LV OK\n"
            f"Current lv1: "
            f"{self._pdb_data.lv_state.lv1_current}\n"
            f"Current lv2: "
            f"{self._pdb_data.lv_state.lv2_current}",
        )
        return stat

    def _hv_diagnostic(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
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

    def _stop_diagnostic(self, stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
        emergency_button_status = self._pdb_data.emergency_button_state
        stop_button_state = self._pdb_data.stop_button_state

        stat.add("Emergency button status", str(emergency_button_status))
        stat.add("Stop button status", str(stop_button_state))

        if emergency_button_status != 1:
            stat.summary(DiagnosticStatus.ERROR, "Emergency button pressed")
        elif stop_button_state != 1:
            stat.summary(DiagnosticStatus.ERROR, "Stop button pressed")
        else:
            stat.summary(DiagnosticStatus.OK, "OK, no stop or emergency button pressed")

        return stat

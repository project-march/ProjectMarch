"""The module motor_controller_state.py contains the CheckMotorControllerStatus Class."""

from typing import List, Callable

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from rclpy.node import Node

from march_shared_msgs.msg import PowerDistributionBoardData


class CheckPDBStatus:
    """Base class to diagnose the motor_controller statuses."""

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

        updater.add(f"PowerDistributionBoard", self._diagnostic())

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
                stat.summary(DiagnosticStatus.STALE, "No more events recorded")
                return stat

            emergency_button_status = self._pdb_data.emergency_button_state
            total_current_state = self._pdb_data.hv_state.total_current
            stop_button_state = self._pdb_data.stop_button_state
            lv1_state = self._pdb_data.lv_state.lv1_ok
            lv2_state = self._pdb_data.lv_state.lv2_ok

            stat.add("Emergency button status", str(emergency_button_status))
            stat.add("Total current state", str(total_current_state))
            stat.add("Stop button status", str(stop_button_state))
            stat.add("LV1 status", str(lv1_state))
            stat.add("LV2 status", str(lv2_state))

            for state in [emergency_button_status, stop_button_state, lv2_state,
                          lv1_state]:
                if state != 1:
                    stat.summary(DiagnosticStatus.ERROR, state)
                    return stat

                stat.summary(DiagnosticStatus.OK, total_current_state)

            return stat

        return d

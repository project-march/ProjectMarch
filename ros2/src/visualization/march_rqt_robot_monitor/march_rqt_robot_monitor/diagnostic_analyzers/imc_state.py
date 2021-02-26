"""The module imc_state.py contains the CheckImcStatus Class."""

from typing import List, Callable

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from rclpy.node import Node

from march_shared_msgs.msg import ImcState


class CheckImcStatus:
    """Base class to diagnose the imc statuses."""

    def __init__(self, node: Node, updater: Updater, joint_names: List[str]):
        """Initialize an IMC diagnostic which analyzes IMC states.

        :type updater: diagnostic_updater.Updater
        """
        self.node = node
        self._sub = node.create_subscription(
            msg_type=ImcState,
            topic="/march/imc_states",
            callback=self._cb,
            qos_profile=10,
        )
        self._imc_state = None

        for i, joint_name in enumerate(joint_names):
            updater.add(f"IMC {joint_name}", self._diagnostic(i))

    def _cb(self, msg: ImcState):
        """Set the imc_states.

        :type msg: ImcState
        """
        self._imc_state = msg

    def _diagnostic(self, index: int) -> Callable:  # noqa: D202
        """Create a diagnostic function for an IMC.

        :type index: int
        :param index: index of the joint

        :return Curried diagnostic function that updates the diagnostic status
                according to the given index.
        """

        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._imc_state is None:
                stat.summary(DiagnosticStatus.STALE, "No more events recorded")
                return stat
            detailed_error = int(self._imc_state.detailed_error[index])
            motion_error = int(self._imc_state.motion_error[index])
            state = self._imc_state.state[index]

            stat.add("Status word", self._imc_state.status_word[index])
            if detailed_error != 0 or motion_error != 0:
                stat.add("Detailed error", self._imc_state.detailed_error[index])
                stat.add(
                    "Detailed error description",
                    self._imc_state.detailed_error_description[index],
                )
                stat.add("Motion error", self._imc_state.motion_error[index])
                stat.add(
                    "Motion error description",
                    self._imc_state.motion_error_description[index],
                )
                stat.summary(DiagnosticStatus.ERROR, state)
            else:
                stat.summary(DiagnosticStatus.OK, state)

            return stat

        return d

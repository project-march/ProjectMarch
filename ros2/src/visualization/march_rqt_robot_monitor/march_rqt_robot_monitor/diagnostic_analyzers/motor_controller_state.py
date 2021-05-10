"""The module motor_controller_state.py contains the CheckMotorControllerStatus Class."""

from typing import List, Callable

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater, DiagnosticStatusWrapper
from rclpy.node import Node

from march_shared_msgs.msg import MotorControllerState


class CheckMotorControllerStatus:
    """Base class to diagnose the motor_controller statuses."""

    def __init__(self, node: Node, updater: Updater, joint_names: List[str]):
        """Initialize an MotorController diagnostic which analyzes MotorController states.

        :type updater: diagnostic_updater.Updater
        """
        self.node = node
        self._sub = node.create_subscription(
            msg_type=MotorControllerState,
            topic="/march/motor_controller_states",
            callback=self._cb,
            qos_profile=10,
        )
        self._motor_controller_state = None

        for i, joint_name in enumerate(joint_names):
            updater.add(f"MotorController {joint_name}", self._diagnostic(i))

    def _cb(self, msg: MotorControllerState):
        """Set the motor_controller_states.

        :type msg: MotorControllerState
        """
        self._motor_controller_state = msg

    def _diagnostic(self, index: int) -> Callable:  # noqa: D202
        """Create a diagnostic function for an MotorController.

        :type index: int
        :param index: index of the joint

        :return Curried diagnostic function that updates the diagnostic status
                according to the given index.
        """

        def d(stat: DiagnosticStatusWrapper) -> DiagnosticStatusWrapper:
            if self._motor_controller_state is None:
                stat.summary(DiagnosticStatus.STALE, "No more events recorded")
                return stat
            error_status = self._motor_controller_state.error_status[index]
            operational_state = self._motor_controller_state.operational_state[index]

            stat.add("Error status", error_status)
            stat.add("Operational state", operational_state)
            if error_status != "":
                stat.summary(DiagnosticStatus.ERROR, operational_state)
            else:
                stat.summary(DiagnosticStatus.OK, operational_state)

            return stat

        return d

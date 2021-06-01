"""The module updater.py contains the DiagnosticUpdater Node."""

import cProfile
import diagnostic_updater
import rclpy
from march_utility.utilities.node_utils import get_joint_names
from rclpy.node import Node
from sensor_msgs.msg import JointState

from march_shared_msgs.msg import Alive

from .diagnostic_analyzers.check_input_device import CheckInputDevice
from .diagnostic_analyzers.control import CheckJointValues
from .diagnostic_analyzers.gait_state import CheckGaitStatus
from .diagnostic_analyzers.motor_controller_state import CheckMotorControllerStatus

NODE_NAME = "rqt_robot_monitor"
HARDWARE_ID = "MARCH VI"

PERIOD = 0.1


class DiagnosticUpdater(Node):
    """This node uses the diagnostic analyzers to provide data for the diagnostic aggregator node."""

    def __init__(self):
        super().__init__(NODE_NAME)

        self.updater = diagnostic_updater.Updater(node=self, period=PERIOD)
        self.updater.setHardwareID(HARDWARE_ID)

        joint_names = get_joint_names(self)

        # Frequency checks
        CheckInputDevice(self, "/march/input_device/alive", Alive, self.updater, 4)

        # Control checks
        check_joint_states = CheckJointValues(self, "/march/joint_states", JointState)
        self.updater.add(
            "Control position values", check_joint_states.position_diagnostics
        )
        self.updater.add(
            "Control velocity values", check_joint_states.velocity_diagnostics
        )
        self.updater.add("Control effort values", check_joint_states.effort_diagnostics)

        # MotorController state check
        CheckMotorControllerStatus(self, self.updater, joint_names)

        # Gait information
        CheckGaitStatus(self, self.updater)

    def update(self):
        """Update the DiagnosticUpdater if there are more than 0 tasks."""
        if len(self.updater.tasks) > 0:
            self.updater.update()

    def start(self):
        """Start the update timer."""
        self.create_timer(PERIOD, self.update)


def main():
    """Start the DiagnosticUpdater Node and the update timer."""
    with cProfile.Profile() as pr:
        rclpy.init()

        node = DiagnosticUpdater()
        node.start()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    pr.dump_stats("march_rqt_robot_monitor")

    rclpy.shutdown()

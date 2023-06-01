"""The module updater.py contains the DiagnosticUpdater Node."""

import diagnostic_updater
import rclpy
from march_utility.utilities.node_utils import get_joint_names_from_service
from rclpy.node import Node
from sensor_msgs.msg import JointState

from march_shared_msgs.msg import Alive

from .diagnostic_analyzers.check_input_device import CheckInputDevice
from .diagnostic_analyzers.control import CheckJointValues
from .diagnostic_analyzers.gait_state import CheckGaitStatus
from .diagnostic_analyzers.motor_controller_state import CheckMotorControllerStatus
from .diagnostic_analyzers.pdb_state import CheckPDBStatus
from ament_index_python.packages import get_package_share_directory
import yaml

from contextlib import suppress

NODE_NAME = "rqt_robot_monitor"
HARDWARE_ID = "MARCH VIII"

PERIOD = 0.1


class DiagnosticUpdater(Node):
    """This node uses the diagnostic analyzers to provide data for the diagnostic aggregator node."""

    def __init__(self):
        super().__init__(NODE_NAME)

        self.updater = diagnostic_updater.Updater(node=self, period=PERIOD)
        self.updater.setHardwareID(HARDWARE_ID)

        robot_path = get_package_share_directory('march_hardware_builder') + '/robots/march7.yaml'
        self.get_logger().info(str(robot_path))
        with open(robot_path) as file:
            document = yaml.full_load(file)
        self.joints = document["march7"]["joints"]

        self.joint_names = []
        for joint in self.joints:
            for name in joint.keys():
                self.joint_names.append(name)

        # Frequency checks
        CheckInputDevice(self, "/march/input_device/alive", Alive, self.updater, 4)

        # Control checks
        check_joint_states = CheckJointValues(self, self.joints, "/joint_states", JointState)
        self.updater.add("Control position values", check_joint_states.position_diagnostics)

        # NOTE: There are also diagnosis for velocity and effort,
        # but since the ROS code does not yet have limits for them, they are not used.

        # MotorController state check
        CheckMotorControllerStatus(self, self.updater, self.joint_names)

        # Gait information
        CheckGaitStatus(self, self.updater)

        # PDB checks
        CheckPDBStatus(self, self.updater)

    def update(self):
        """Update the DiagnosticUpdater if there are more than 0 tasks."""
        if len(self.updater.tasks) > 0:
            self.updater.update()

    def start(self):
        """Start the update timer."""
        self.create_timer(PERIOD, self.update)


def main():
    """Start the DiagnosticUpdater Node and the update timer."""
    rclpy.init()

    node = DiagnosticUpdater()
    node.start()

    with suppress(KeyboardInterrupt):
        rclpy.spin(node)

    rclpy.shutdown()

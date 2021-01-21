import diagnostic_updater
import rclpy
from march_utility.utilities.node_utils import get_joint_names
from rclpy.node import Node
from sensor_msgs.msg import JointState

from march_shared_msgs.msg import Alive

from .diagnostic_analyzers.check_input_device import CheckInputDevice
from .diagnostic_analyzers.control import CheckJointValues
from .diagnostic_analyzers.gait_state import CheckGaitStatus
from .diagnostic_analyzers.imc_state import CheckImcStatus

NODE_NAME = 'rqt_robot_monitor'
HARDWARE_ID = 'MARCH VI'

FREQUENCY = 10
PERIOD = 1 / FREQUENCY


class DiagnosticUpdater(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.updater = diagnostic_updater.Updater(node=self, period=PERIOD)
        self.updater.setHardwareID(HARDWARE_ID)

        joint_names = get_joint_names(self)

        # Frequency checks
        CheckInputDevice(self, '/march/input_device/alive', Alive, self.updater, 5)

        # Control checks
        # check_joint_states = CheckJointValues(self,
        #                                                  '/march/joint_states',
        #                                                  JointState)
        # self.updater.add('Control position values',
        #                  check_joint_states.position_diagnostics)
        # self.updater.add('Control velocity values',
        #                  check_joint_states.velocity_diagnostics)
        # self.updater.add('Control effort values',
        #                  check_joint_states.effort_diagnostics)

        # IMC state check
        CheckImcStatus(self, self.updater, joint_names)

        # Gait information
        CheckGaitStatus(self, self.updater)

        # self.updater.force_update()

    def update(self):


        if len(self.updater.tasks) == 0:
            self.get_logger().warn("No tasks found!")
        else:
            # self.get_logger().info(f'Names: {[task.name for task in self.updater.tasks]}')
            self.updater.update()

    def start(self):
        self.create_timer(PERIOD, self.update)


def main():
    rclpy.init()

    node = DiagnosticUpdater()

    node.start()

    rclpy.spin(node)

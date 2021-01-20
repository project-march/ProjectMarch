import diagnostic_updater
import rclpy
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

        # Frequency checks
        # CheckInputDevice(self, '/march/input_device/alive', Alive, self.updater, 5)
        #
        # Control checks
        check_current_movement_values = CheckJointValues(self,
                                                         'march/joint_states',
                                                         JointState)
        self.updater.add('Control position values',
                         check_current_movement_values.position_diagnostics)
        # self.updater.add('Control velocity values',
        #                  check_current_movement_values.velocity_diagnostics)
        # self.updater.add('Control effort values',
        #                  check_current_movement_values.effort_diagnostics)
        #
        # # IMC state check
        # CheckImcStatus(self, self.updater)
        #
        # # Gait information
        # CheckGaitStatus(self, self.updater)

        # self.updater.force_update()

    def start(self):
        update_rate = self.create_rate(FREQUENCY)
        while rclpy.ok():
            update_rate.sleep()
            self.updater.update()


def main():
    rclpy.init()

    node = DiagnosticUpdater()

    node.start()

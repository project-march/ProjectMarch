import diagnostic_updater
import rospy
from sensor_msgs.msg import JointState

from march_shared_msgs.msg import Alive

from .diagnostic_analyzers.check_input_device import CheckInputDevice
from .diagnostic_analyzers.control import CheckJointValues
from .diagnostic_analyzers.gait_state import CheckGaitStatus
from .diagnostic_analyzers.imc_state import CheckImcStatus


def main():
    rospy.init_node("Diagnostic_updater")

    updater = diagnostic_updater.Updater()
    updater.setHardwareID("MARCH IVc")

    # Frequency checks
    CheckInputDevice("/march/input_device/alive", Alive, updater, 5)

    # control checks
    check_current_movement_values = CheckJointValues("march/joint_states", JointState)
    updater.add(
        "Control position values", check_current_movement_values.position_diagnostics
    )
    updater.add(
        "Control velocity values", check_current_movement_values.velocity_diagnostics
    )
    updater.add(
        "Control effort values", check_current_movement_values.effort_diagnostics
    )

    # IMC state check
    CheckImcStatus(updater)

    # Gait information
    CheckGaitStatus(updater)

    updater.force_update()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        updater.update()

import diagnostic_updater
import rospy
from sensor_msgs.msg import JointState, Temperature

from march_shared_resources.msg import Alive

from .diagnostic_analyzers.check_input_device import CheckInputDevice
from .diagnostic_analyzers.control import CheckJointValues
from .diagnostic_analyzers.gait_state import CheckGaitStatus
from .diagnostic_analyzers.imc_state import CheckImcStatus
from .diagnostic_analyzers.temperature import CheckJointTemperature


def main():
    rospy.init_node('Diagnostic_updater')

    updater = diagnostic_updater.Updater()
    updater.setHardwareID('MARCH IVc')

    # Frequency checks
    CheckInputDevice('/march/input_device/alive', Alive, updater, 5)

    # Temperature checks
    check_temp_joint_left_ankle = \
        CheckJointTemperature('Temperature left ankle', '/march/temperature/left_ankle', Temperature)
    updater.add('Temperature left ankle', check_temp_joint_left_ankle.diagnostics)

    check_temp_joint_left_knee = \
        CheckJointTemperature('Temperature left knee', '/march/temperature/left_knee', Temperature)
    updater.add('Temperature left knee', check_temp_joint_left_knee.diagnostics)

    check_temp_joint_left_hip_fe = \
        CheckJointTemperature('Temperature left hip FE', '/march/temperature/left_hip_fe', Temperature)
    updater.add('Temperature left hip FE', check_temp_joint_left_hip_fe.diagnostics)

    check_temp_joint_left_hip_aa = \
        CheckJointTemperature('Temperature left hip AA', '/march/temperature/left_hip_aa', Temperature)
    updater.add('Temperature left hip AA', check_temp_joint_left_hip_aa.diagnostics)

    check_temp_joint_right_ankle = \
        CheckJointTemperature('Temperature right ankle', '/march/temperature/right_ankle', Temperature)
    updater.add('Temperature right ankle', check_temp_joint_right_ankle.diagnostics)

    check_temp_joint_right_knee = \
        CheckJointTemperature('Temperature right knee', '/march/temperature/right_knee', Temperature)
    updater.add('Temperature right knee', check_temp_joint_right_knee.diagnostics)

    check_temp_joint_right_hip_fe = \
        CheckJointTemperature('Temperature right hip FE', '/march/temperature/right_hip_fe', Temperature)
    updater.add('Temperature right hip FE', check_temp_joint_right_hip_fe.diagnostics)

    check_temp_joint_right_hip_aa = \
        CheckJointTemperature('Temperature right hip AA', '/march/temperature/right_hip_aa', Temperature)
    updater.add('Temperature right hip AA', check_temp_joint_right_hip_aa.diagnostics)

    # control checks
    check_current_movement_values = CheckJointValues('march/joint_states', JointState)
    updater.add('Control position values', check_current_movement_values.position_diagnostics)
    updater.add('Control velocity values', check_current_movement_values.velocity_diagnostics)
    updater.add('Control effort values', check_current_movement_values.effort_diagnostics)

    # IMC state check
    CheckImcStatus(updater)

    # Gait information
    CheckGaitStatus(updater)

    updater.force_update()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        updater.update()

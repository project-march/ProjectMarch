
import diagnostic_updater
import rospy
from std_msgs.msg import Time

from .diagnostic_analyzers.temperature import temperature_updater
from .diagnostic_analyzers.topic_frequency import CheckTopicFrequency


def main():
    rospy.init_node('Diagnostic_updater')

    updater = diagnostic_updater.Updater()
    updater.setHardwareID('MARCH IVc')

    CheckTopicFrequency('Input_Device', '/march/input_device/alive', Time, updater, 5)
    temperature_updater(updater)

    updater.force_update()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        updater.update()

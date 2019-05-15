import os
import rospy

import yaml

from UserInterfaceController import notify
import GaitFactory
from march_rqt_gait_generator.msg import MarchGait
from rospy_message_converter import message_converter


def export_to_file(gait, gait_directory):

    march_gait = MarchGait()
    march_gait.joint_trajectory = gait.to_joint_trajectory()
    march_gait.actual_setpoints = gait.to_actual_setpoints()
    march_gait.gait = gait.name
    march_gait.subgait = "Subgait placeholder"
    march_gait.version = gait.version
    march_gait.description = gait.description

    march_gait.duration = rospy.Duration.from_sec(gait.duration)

    output_file_directory = os.path.join(gait_directory, gait.name.replace(" ", "_"))
    output_file_path = os.path.join(output_file_directory, gait.version.replace(" ", "_") + ".gait")

    rospy.loginfo("Writing gait to " + output_file_path)

    try:
        os.makedirs(output_file_directory)
    except OSError:
        if not os.path.isdir(output_file_directory):
            raise

    output_file = open(output_file_path, 'w')
    output_file.write(str(march_gait))

    rospy.logwarn(march_gait)

    notify("Gait Saved", output_file_path)

    output_file.close()


def import_from_file_name(robot, file_name):
    march_gait_yaml = yaml.load(open(file_name))
    march_gait = message_converter.convert_dictionary_to_ros_message('march_shared_resources/Subgait', march_gait_yaml)
    return GaitFactory.from_msg(robot, march_gait)

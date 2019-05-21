import os
import rospy

import yaml

from UserInterfaceController import notify
import GaitFactory
from march_shared_resources.msg import Subgait
from rospy_message_converter import message_converter


def export_to_file(gait, gait_directory):

    # Name and version will be empty as it's stored in the filename.
    subgait = Subgait()

    subgait.trajectory = gait.to_joint_trajectory()
    subgait.setpoints = gait.to_setpoints()
    subgait.description = gait.description

    subgait.duration = rospy.Duration.from_sec(gait.duration)

    output_file_directory = os.path.join(gait_directory, gait.name.replace(" ", "_"), gait.subgait.replace(" ", "_"))
    output_file_path = os.path.join(output_file_directory, gait.version.replace(" ", "_") + ".subgait")

    rospy.loginfo("Writing gait to " + output_file_path)

    try:
        os.makedirs(output_file_directory)
    except OSError:
        if not os.path.isdir(output_file_directory):
            raise

    output_file = open(output_file_path, 'w')
    output_file.write(str(subgait))

    notify("Gait Saved", output_file_path)

    output_file.close()


def import_from_file_name(robot, file_name):
    gait_name = file_name.split("/")[-3]
    subgait_name = file_name.split("/")[-2]
    version = file_name.split("/")[-1].replace(".subgait", "")
    march_subgait_yaml = yaml.load(open(file_name))
    march_subgait = message_converter.convert_dictionary_to_ros_message(
        'march_shared_resources/Subgait', march_subgait_yaml)
    return GaitFactory.from_msg(robot, march_subgait, gait_name, subgait_name, version)

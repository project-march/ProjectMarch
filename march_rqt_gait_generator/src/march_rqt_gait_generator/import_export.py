import os
import rospy
from UserInterfaceController import notify


def export_to_file(gait, gait_directory):
    joint_trajectory = gait.to_joint_trajectory()
    output_file_directory = os.path.join(gait_directory, gait.name.replace(" ", "_"))
    output_file_path = os.path.join(output_file_directory, gait.version.replace(" ", "_") + ".gait")

    rospy.loginfo("Writing gait to " + output_file_path)

    try:
        os.makedirs(output_file_directory)
    except OSError:
        if not os.path.isdir(output_file_directory):
            raise

    output_file = open(output_file_path, 'w')
    output_file.write(str(joint_trajectory))

    notify("Gait Saved", output_file_path)

    output_file.close()

import os
import rospy

from UserInterfaceController import notify
from march_shared_resources.msg import Subgait
from python_qt_binding.QtWidgets import QMessageBox


def export_to_file(gait, gait_directory):
    if gait_directory is None or gait_directory == "":
        return

    # Name and version will be empty as it's stored in the filename.
    subgait = Subgait()

    subgait.gait_type = gait.gait_type
    subgait.trajectory = gait.to_joint_trajectory_msg()
    subgait.setpoints = gait.to_setpoints()
    subgait.description = str(gait.description)

    subgait.duration = rospy.Duration.from_sec(gait.duration)

    output_file_directory = os.path.join(gait_directory,
                                         gait.gait_name.replace(" ", "_"),
                                         gait.subgait_name.replace(" ", "_"))
    output_file_path = os.path.join(output_file_directory,
                                    gait.version.replace(" ", "_") + ".subgait")

    file_exists = os.path.isfile(output_file_path)
    if file_exists:
        overwrite_file = QMessageBox.question(None, 'File already exists',
                                              "Do you want to overwrite " + str(output_file_path) + "?",
                                              QMessageBox.Yes | QMessageBox.No)
        if overwrite_file == QMessageBox.No:
            return

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

import math
import os
import subprocess

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import (
    QFileDialog,
    QFrame,
    QHeaderView,
    QMessageBox,
    QShortcut,
    QWidget,
)
import rospkg
import rospy
from rviz import bindings as rviz
from sensor_msgs.msg import JointState
from tf import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)


class InverseKinematicsSetpointsView(QWidget):
    def __init__(self, parent=None):
        super(InverseKinematicsSetpointsView, self).__init__(parent)

        ui_file = os.path.join(
            rospkg.RosPack().get_path("march_rqt_gait_generator"),
            "resource",
            "inverse_kinematics_setpoints_input.ui",
        )
        loadUi(ui_file, self)

        self.z_axis_combo_box.addItems(["from hip downwards", "from lowest point upwards"])

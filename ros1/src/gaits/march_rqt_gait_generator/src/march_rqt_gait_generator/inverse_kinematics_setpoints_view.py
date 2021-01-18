from PyQt5 import QtCore, QtGui, QtWidgets

from .inverse_kinematics_setpoints_input import Ui_MainWindow


class InverseKinematicsSetpointsView(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(InverseKinematicsSetpointsView, self).__init__(parent)
        self.setupUi(self)

        self.z_axis_combo_box.addItems(["from hip downwards", "from lowest point upwards"])

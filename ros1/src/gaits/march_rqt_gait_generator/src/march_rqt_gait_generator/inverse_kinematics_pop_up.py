from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog
from python_qt_binding import loadUi
from march_shared_classes.utilities.vector_3d import Vector3d
from march_shared_classes.utilities.side import Side


class InverseKinematicsPopUpWindow(QDialog):
    """A pop up window for retrieving the inputs for the inverse kinematics setpoints feature."""
    
    def __init__(self, parent, ui_file: str):
        """Connects the ok and cancel buttons."""
        super(InverseKinematicsPopUpWindow, self).__init__(
            parent=parent, flags=Qt.Window
        )
        loadUi(ui_file, self)

        self.buttonBox.accepted.connect(self.save)
        self.buttonBox.rejected.connect(self.cancel)

    def show_pop_up(self):
        """Show pop up."""
        self.foot_side = ""
        self.z_axis = ""
        self.use_default_y_position = True
        self.position_input = Vector3d(0, 0, 0)
        self.velocity_input = Vector3d(0, 0, 0)
        self.time = 0
        self.cancelled = False
        return super(InverseKinematicsPopUpWindow, self).exec_()

    def cancel(self):
        """Close without applying the values."""
        self.cancelled = True
        self.reject()

    def save(self):
        """Check and save value while closing, close if successful."""
        if self.footSideComboBox.currentText() == "Right":
            self.foot_side = Side.right
        else:
            self.foot_side = Side.left

        self.z_axis = self.zAxisComboBox.currentText()
        self.use_default_y_position = self.useDefaultYComboBox.currentText() == "yes"
        self.position_input = (
            Vector3d(
                self.xCoordinateSpinBox.value(),
                self.yCoordinateSpinBox.value(),
                self.zCoordinateSpinBox.value(),
            )
            / 100
        )
        self.velocity_input = (
            Vector3d(
                self.xVelocitySpinBox.value(),
                self.yVelocitySpinBox.value(),
                self.zVelocitySpinBox.value(),
            )
            / 100
        )
        self.time = self.timeSpinBox.value()
        self.accept()

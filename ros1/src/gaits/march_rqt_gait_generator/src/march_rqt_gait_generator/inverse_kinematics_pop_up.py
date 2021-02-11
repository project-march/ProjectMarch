from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from march_shared_classes.utilities.vector_3d import Vector3d
from march_shared_classes.utilities.side import Side


class InverseKinematicsPopUpWindow(QDialog):
    """Retrieves the inputs for the inverse kinematics setpoints feature with a pop up."""

    def __init__(self, parent: QWidget, ui_file: str) -> None:
        """Connects the ok and cancel buttons."""
        super(InverseKinematicsPopUpWindow, self).__init__(
            parent=parent, flags=Qt.Window
        )
        loadUi(ui_file, self)

        self.buttonBox.accepted.connect(self.save)
        self.buttonBox.rejected.connect(self.cancel)

    def show_pop_up(self) -> any:
        """Show pop up."""
        self.foot_side = ""
        self.z_axis = ""
        self.position_input = Vector3d(0, 0, 0)
        self.velocity_input = Vector3d(0, 0, 0)
        self.time = 0
        self.cancelled = False
        return super(InverseKinematicsPopUpWindow, self).exec_()

    def cancel(self) -> None:
        """Close without applying the values."""
        self.cancelled = True
        self.reject()

    def save(self) -> None:
        """Check and save value while closing, close if successful."""
        if self.footSideComboBox.currentText() == "Right":
            self.foot_side = Side.right
        else:
            self.foot_side = Side.left

        self.z_axis = self.zAxisComboBox.currentText()
        self.position_input = (
            Vector3d(
                self.xCoordinateSpinBox.value(),
                self.yCoordinateSpinBox.value(),
                self.zCoordinateSpinBox.value(),
            )
            / 100  # Convert the input in cm to meters for calculation
        )
        self.velocity_input = (
            Vector3d(
                self.xVelocitySpinBox.value(),
                self.yVelocitySpinBox.value(),
                self.zVelocitySpinBox.value(),
            )
            / 100  # Convert the input in cm / s to meters / s for calculation
        )
        self.time = self.timeSpinBox.value()
        self.accept()

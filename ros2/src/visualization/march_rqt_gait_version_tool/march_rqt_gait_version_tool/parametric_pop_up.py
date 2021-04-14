from PyQt5 import QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog
from python_qt_binding import loadUi


class ParametricPopUpWindow(QDialog):
    def __init__(self, parent, ui_file):
        """A pop up window for retrieving a base version, other version and parameter for a parametric subgait.

        :param parent:
            The parent widget to connect to the pop up
        :param width:
            Starting width of the the pop up widget
        :param height:
            Starting height of the the pop up widget
        """
        super(ParametricPopUpWindow, self).__init__(parent=parent, flags=Qt.Window)
        loadUi(ui_file, self)

        self.buttonBox.accepted.connect(self.save)
        self.buttonBox.rejected.connect(self.cancel)
        self.firstParameterSlider.valueChanged.connect(
            self.first_parameter_value_changed
        )
        self.secondParameterSlider.valueChanged.connect(
            self.second_parameter_value_changed
        )
        self.fourSubgaitInterpolation.stateChanged.connect(
            self.four_subgait_interpolation_changed
        )

        # Initialize the third and fourth combo box and the second parameter slider as blocked
        self.thirdVersionComboBox.setEnabled(False)
        self.fourthVersionComboBox.setEnabled(False)
        self.secondParameterSlider.setEnabled(False)

    def show_pop_up(self, versions):
        """Reset and show pop up."""
        self.firstVersionComboBox.clear()
        self.firstVersionComboBox.addItems(versions)
        self.secondVersionComboBox.clear()
        self.secondVersionComboBox.addItems(versions)
        self.thirdVersionComboBox.clear()
        self.thirdVersionComboBox.addItems(versions)
        self.fourthVersionComboBox.clear()
        self.fourthVersionComboBox.addItems(versions)
        self.firstParameterSlider.setValue(50)
        self.firstParameterLabel.setText("first parameter = 0.50")
        self.secondParameterSlider.setValue(50)
        self.secondParameterLabel.setText("second parameter = 0.50")

        self.four_subgait_interpolation = False

        # For 'normal' parametric gaits between two subgaits
        self.parameter = 0.0
        self.base_version = ""
        self.other_version = ""

        # For 'multiple' parametric gaits between four subgaits
        self.first_parameter = 0.0
        self.second_parameter = 0.0
        self.first_version = ""
        self.second_version = ""
        self.third_version = ""
        self.fourth_version = ""
        return super(ParametricPopUpWindow, self).exec_()

    def first_parameter_value_changed(self):
        """Puts the new slider value in the label next to it."""
        self.firstParameterLabel.setText(
            "first parameter = {val:.2f}".format(
                val=self.firstParameterSlider.value() / 100.0
            )
        )

    def second_parameter_value_changed(self):
        """Puts the new slider value in the label next to it."""
        self.secondParameterLabel.setText(
            "second parameter = {val:.2f}".format(
                val=self.secondParameterSlider.value() / 100.0
            )
        )

    def four_subgait_interpolation_changed(self):
        """Unlocks the buttons for four subgait interpolation when it is enabled"""
        if self.fourSubgaitInterpolation.isChecked():
            self.thirdVersionComboBox.setEnabled(True)
            self.fourthVersionComboBox.setEnabled(True)
            self.secondParameterSlider.setEnabled(True)
        else:
            self.thirdVersionComboBox.setEnabled(False)
            self.fourthVersionComboBox.setEnabled(False)
            self.secondParameterSlider.setEnabled(False)

    def cancel(self):
        """Close without applying the values."""
        self.reject()

    def save(self):
        """Check and save value while closing, close if successful."""
        self.four_subgait_interpolation = self.fourSubgaitInterpolation.isChecked()
        if self.four_subgait_interpolation:
            self.first_version = self.firstVersionComboBox.currentText()
            self.second_version = self.secondVersionComboBox.currentText()
            self.third_version = self.thirdVersionComboBox.currentText()
            self.fourth_version = self.fourthVersionComboBox.currentText()
            self.first_parameter = self.firstParameterSlider.value() / 100.0
            self.second_parameter = self.secondParameterSlider.value() / 100.0
            self.accept()
        else:
            self.base_version = self.firstVersionComboBox.currentText()
            self.other_version = self.secondVersionComboBox.currentText()
            self.parameter = self.firstParameterSlider.value() / 100.0
            self.accept()

from typing import List

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
            lambda: ParametricPopUpWindow.first_parameter_value_changed(self)
        )
        self.secondParameterSlider.valueChanged.connect(
            lambda: ParametricPopUpWindow.second_parameter_value_changed(self)
        )
        self.fourSubgaitInterpolation.stateChanged.connect(
            lambda: self.four_subgait_interpolation_changed()
        )
        self.set_second_parameterize_enabled(False)

        self.uses_four_subgait_interpolation = False
        self.parameters: List[float] = []
        self.selected_versions: List[str] = []

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
            self.set_second_parameterize_enabled(True)
        else:
            self.set_second_parameterize_enabled(False)

    def set_second_parameterize_enabled(self, value: bool):
        self.thirdVersionComboBox.setEnabled(value)
        self.fourthVersionComboBox.setEnabled(value)
        self.secondParameterSlider.setEnabled(value)

    def cancel(self):
        """Close without applying the values."""
        self.reject()

    def save(self):
        """Check and save value while closing, close if successful."""
        self.uses_four_subgait_interpolation = self.fourSubgaitInterpolation.isChecked()

        self.parameters = [self.firstParameterSlider.value() / 100.0]
        self.selected_versions = [
            self.firstVersionComboBox.currentText(),
            self.secondVersionComboBox.currentText(),
        ]

        if self.four_subgait_interpolation:
            self.parameters.append(self.secondParameterSlider.value() / 100.0)
            self.selected_versions.append(self.thirdVersionComboBox.currentText())
            self.selected_versions.append(self.fourthVersionComboBox.currentText())

        self.accept()

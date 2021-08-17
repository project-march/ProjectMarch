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

    @staticmethod
    def first_parameter_value_changed(parametric_pop_up):
        """Puts the new slider value in the label next to it."""
        parametric_pop_up.firstParameterLabel.setText(
            "first parameter = {val:.2f}".format(
                val=parametric_pop_up.firstParameterSlider.value() / 100.0
            )
        )

    @staticmethod
    def second_parameter_value_changed(parametric_pop_up):
        """Puts the new slider value in the label next to it."""
        parametric_pop_up.secondParameterLabel.setText(
            "second parameter = {val:.2f}".format(
                val=parametric_pop_up.secondParameterSlider.value() / 100.0
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

import re

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog
from python_qt_binding import loadUi
from .parametric_pop_up import ParametricPopUpWindow
from .subgait_version_select import select_same_subgait_versions

NUM_SECTIONS = 4


class ParametricSameVersionsPopUpWindow(QDialog):
    def __init__(self, parent, ui_file):
        super(ParametricSameVersionsPopUpWindow, self).__init__(
            parent=parent, flags=Qt.Window
        )
        loadUi(ui_file, self)

        self.buttonBox.accepted.connect(self.ok)
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

        # Connect subgaitButtonBoxes to selecting same versions
        # for i in range(1, 5):
        #     self.__getattribute__(f"subgaitButtonBox{i}").accepted.connect(lambda: self._select_same_versions(i))
        #     self.__getattribute__(f"subgaitButtonBox{i}").rejected.connect(lambda: self._clear_subgait_selection(i))

        self.subgaitButtonBox1.accepted.connect(lambda: self._select_same_versions(1))
        self.subgaitButtonBox1.rejected.connect(
            lambda: self._clear_subgait_selection(1)
        )
        self.subgaitButtonBox2.accepted.connect(lambda: self._select_same_versions(2))
        self.subgaitButtonBox2.rejected.connect(
            lambda: self._clear_subgait_selection(2)
        )
        self.subgaitButtonBox3.accepted.connect(lambda: self._select_same_versions(3))
        self.subgaitButtonBox3.rejected.connect(
            lambda: self._clear_subgait_selection(3)
        )
        self.subgaitButtonBox4.accepted.connect(lambda: self._select_same_versions(4))
        self.subgaitButtonBox4.rejected.connect(
            lambda: self._clear_subgait_selection(4)
        )

        self.resetAllButton.clicked.connect(self._reset_all)

        self.set_second_parameterize_enabled(False)

        self.gait = ""
        self.subgaits = {}

        self._init_sliders()

    def _init_sliders(self, value: float = 50):
        self.firstParameterSlider.setValue(value)
        self.firstParameterLabel.setText(
            f"first parameter = {self.firstParameterSlider.value() / 100}"
        )
        self.secondParameterSlider.setValue(value)
        self.secondParameterLabel.setText(
            f"second parameter = {self.secondParameterSlider.value() / 100}"
        )

    def show_pop_up(self, gait: str, subgaits: dict):
        """Reset and show pop up."""
        self.gait = gait
        self.subgaits = subgaits

        return super(ParametricSameVersionsPopUpWindow, self).exec_()

    def cancel(self):
        """Close without applying the values."""
        self.reject()

    def ok(self):
        """Save value while closing."""
        self.accept()

    def four_subgait_interpolation_changed(self):
        """Unlocks the buttons for four subgait interpolation when it is enabled"""
        if self.fourSubgaitInterpolation.isChecked():
            self.set_second_parameterize_enabled(True)
        else:
            self.set_second_parameterize_enabled(False)

    def set_second_parameterize_enabled(self, value: bool):
        self.prefix_input3.setEnabled(value)
        self.prefix_input4.setEnabled(value)
        self.postfix_input3.setEnabled(value)
        self.postfix_input4.setEnabled(value)
        self.subgaitButtonBox3.setEnabled(value)
        self.subgaitButtonBox4.setEnabled(value)
        self.selectedSubgaits3.setEnabled(value)
        self.selectedSubgaits4.setEnabled(value)
        self.secondParameterSlider.setEnabled(value)

    def _select_same_versions(self, index: int):
        prefix = self.get_subgait_prefix(index)
        postfix = self.get_subgait_postfix(index)

        selected_versions = select_same_subgait_versions(
            self.gait, self.subgaits, prefix, postfix
        )

        output = str(selected_versions)
        self.__getattribute__(f"selectedSubgaits{index}").setText(output)

    def get_subgait_prefix(self, index: int):
        return self.__getattribute__(f"prefix_input{index}").text()

    def get_subgait_postfix(self, index: int):
        return self.__getattribute__(f"postfix_input{index}").text()

    def _clear_subgait_selection(self, index: int):
        self.__getattribute__(f"prefix_input{index}").setText("")
        self.__getattribute__(f"postfix_input{index}").setText("")
        self.__getattribute__(f"selectedSubgaits{index}").setText("")

    def _reset_all(self):
        for i in range(NUM_SECTIONS):
            self._clear_subgait_selection(i + 1)
        self._init_sliders()

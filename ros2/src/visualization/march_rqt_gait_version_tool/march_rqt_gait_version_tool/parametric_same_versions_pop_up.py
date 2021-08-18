import ast
from typing import Dict, List

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog, QTextBrowser, QDialogButtonBox, QLineEdit
from python_qt_binding import loadUi
from .parametric_pop_up import ParametricPopUpWindow
from .subgait_version_select import select_same_subgait_versions

NUM_SECTIONS = 4


class ParseException(Exception):
    def __init__(self, msg: str):
        super().__init__(f"Something went wrong while parsing input: {msg}")


class ParseSubgaitException(ParseException):
    def __init__(self, index: int):
        super().__init__(f"Unable to parse selected subgaits of box {index}")


class ParametricSameVersionsPopUpWindow(QDialog):
    def __init__(self, parent, ui_file):
        super(ParametricSameVersionsPopUpWindow, self).__init__(
            parent=parent, flags=Qt.Window
        )
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
        self.uses_four_subgait_interpolation = False
        self.parameters: List[float] = []
        self.all_selected_versions: List[Dict[str, str]] = []

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

    def save(self):
        """Save value while closing."""
        try:
            self.uses_four_subgait_interpolation = (
                self.fourSubgaitInterpolation.isChecked()
            )

            self.parameters = [self.firstParameterSlider.value() / 100]
            self.all_selected_versions = [
                self._get_selected_subgaits(1),
                self._get_selected_subgaits(2),
            ]

            if self.uses_four_subgait_interpolation:
                self.parameters.append(self.secondParameterSlider.value() / 100)
                self.all_selected_versions.append(self._get_selected_subgaits(3))
                self.all_selected_versions.append(self._get_selected_subgaits(4))

            self.accept()
        except ParseException:
            self.reject()

    def four_subgait_interpolation_changed(self):
        """Unlocks the buttons for four subgait interpolation when it is enabled"""
        if self.fourSubgaitInterpolation.isChecked():
            self.set_second_parameterize_enabled(True)
        else:
            self.set_second_parameterize_enabled(False)

    def set_second_parameterize_enabled(self, value: bool):
        self.get_prefix_input_line_edit(3).setEnabled(value)
        self.get_prefix_input_line_edit(4).setEnabled(value)
        self.get_postfix_input_line_edit(3).setEnabled(value)
        self.get_postfix_input_line_edit(4).setEnabled(value)
        self._get_subgait_button_box(3).setEnabled(value)
        self._get_subgait_button_box(4).setEnabled(value)
        self._get_selected_subgaits_text_browser(3).setEnabled(value)
        self._get_selected_subgaits_text_browser(4).setEnabled(value)
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
        self.get_prefix_input_line_edit(index).setText("")
        self.get_postfix_input_line_edit(index).setText("")
        self._get_selected_subgaits_text_browser(index).setText("")

    def _get_selected_subgaits_text_browser(self, index) -> QTextBrowser:
        return self.__getattribute__(f"selectedSubgaits{index}")

    def _get_subgait_button_box(self, index) -> QDialogButtonBox:
        return self.__getattribute__(f"subgaitButtonBox{index}")

    def get_prefix_input_line_edit(self, index) -> QLineEdit:
        return self.__getattribute__(f"prefix_input{index}")

    def get_postfix_input_line_edit(self, index) -> QLineEdit:
        return self.__getattribute__(f"postfix_input{index}")

    def _get_selected_subgaits(self, index: int) -> dict:
        text = self._get_selected_subgaits_text_browser(index).toPlainText()

        if text == "":
            raise ParseSubgaitException(index)

        try:
            selected_subgaits = ast.literal_eval(text)
        except SyntaxError:
            raise ParseSubgaitException(index)

        if not isinstance(selected_subgaits, dict):
            raise ParseSubgaitException(index)

        return selected_subgaits

    def _reset_all(self):
        for i in range(NUM_SECTIONS):
            self._clear_subgait_selection(i + 1)
        self._init_sliders()

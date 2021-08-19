import ast
from typing import Dict, List

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDialog,
    QTextBrowser,
    QDialogButtonBox,
    QLineEdit,
    QLabel,
    QSlider,
    QWidget,
)
from python_qt_binding import loadUi

from .subgait_version_select import select_same_subgait_versions

NUM_SECTIONS = 4


class ParseError(Exception):
    def __init__(self, msg: str):
        super().__init__(f"Something went wrong while parsing input: {msg}")


class ParseSubgaitException(ParseError):
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
        self.buttonBox.button(QDialogButtonBox.Ok).setEnabled(False)

        # Connect parameter sliders to parameter labels
        self._get_parameter_slider(1).valueChanged.connect(
            lambda: self._parameter_value_changed(1)
        )
        self._get_parameter_slider(2).valueChanged.connect(
            lambda: self._parameter_value_changed(2)
        )

        self.fourSubgaitInterpolation.stateChanged.connect(
            lambda: self._four_subgait_interpolation_changed()
        )

        # Connect subgaitButtonBoxes to selecting same versions
        self._get_subgait_button_box(1).accepted.connect(
            lambda: self._select_same_versions(1)
        )
        self._get_subgait_button_box(1).rejected.connect(
            lambda: self._clear_subgait_selection(1)
        )
        self._get_subgait_button_box(2).accepted.connect(
            lambda: self._select_same_versions(2)
        )
        self._get_subgait_button_box(2).rejected.connect(
            lambda: self._clear_subgait_selection(2)
        )
        self._get_subgait_button_box(3).accepted.connect(
            lambda: self._select_same_versions(3)
        )
        self._get_subgait_button_box(3).rejected.connect(
            lambda: self._clear_subgait_selection(3)
        )
        self._get_subgait_button_box(4).accepted.connect(
            lambda: self._select_same_versions(4)
        )
        self._get_subgait_button_box(4).rejected.connect(
            lambda: self._clear_subgait_selection(4)
        )

        self.resetAllButton.clicked.connect(self._reset_all)

        self.gait = ""
        self.subgaits = {}
        self.uses_four_subgait_interpolation = False
        self.parameters: List[float] = []
        self.all_selected_versions: List[Dict[str, str]] = []

        self.set_second_parameterize_enabled(False)
        self._init_sliders()

    def _init_sliders(self, value: float = 50):
        self._get_parameter_slider(1).setValue(value)
        self._get_parameter_slider(2).setValue(value)

        value_ratio = value / 100
        self._get_parameter_label(1).setText(f"first parameter = {value_ratio}")
        self._get_parameter_label(2).setText(f"second parameter = {value_ratio}")

    def show_pop_up(self, gait: str, subgaits: dict):
        """Reset and show pop up."""
        self.gait = gait
        self.subgaits = subgaits

        self.allVersions.setText(str(self.subgaits))

        return super(ParametricSameVersionsPopUpWindow, self).exec_()

    def cancel(self):
        """Close without applying the values."""
        self.reject()

    def save(self):
        """Save value while closing."""
        try:
            self.parameters = [self._get_parameter_slider(1).value() / 100]
            self.all_selected_versions = [
                self._get_selected_subgaits(1),
                self._get_selected_subgaits(2),
            ]

            if self.uses_four_subgait_interpolation:
                self.parameters.append(self._get_parameter_slider(2).value() / 100)
                self.all_selected_versions.append(self._get_selected_subgaits(3))
                self.all_selected_versions.append(self._get_selected_subgaits(4))

            self.accept()
        except ParseError:
            self.reject()

    def _parameter_value_changed(self, index: int):
        """Puts the new slider value in the label next to it."""
        if index == 1:
            index_str = "first"
        elif index == 2:
            index_str = "second"
        else:
            return
        self._get_parameter_label(index).setText(
            f"{index_str} parameter = {self._get_parameter_slider(index).value() / 100:.2f}"
        )

    def _four_subgait_interpolation_changed(self):
        """Unlocks the buttons for four subgait interpolation when it is enabled"""
        self.uses_four_subgait_interpolation = self.fourSubgaitInterpolation.isChecked()
        if self.uses_four_subgait_interpolation:
            self.set_second_parameterize_enabled(True)
        else:
            self.set_second_parameterize_enabled(False)
        self._verify_input()

    def set_second_parameterize_enabled(self, value: bool):
        widgets: List[QWidget] = [
            self._get_prefix_input_line_edit(3),
            self._get_prefix_input_line_edit(4),
            self._get_postfix_input_line_edit(3),
            self._get_postfix_input_line_edit(4),
            self._get_subgait_button_box(3),
            self._get_subgait_button_box(4),
            self._get_selected_subgaits_text_browser(3),
            self._get_selected_subgaits_text_browser(4),
            self._get_parameter_slider(2),
        ]

        for widget in widgets:
            widget.setEnabled(value)

    def _select_same_versions(self, index: int):
        prefix = self.get_subgait_prefix(index)
        postfix = self.get_subgait_postfix(index)

        if not (prefix == "" and postfix == ""):
            selected_versions = select_same_subgait_versions(
                self.gait, self.subgaits, prefix, postfix
            )

            if len(selected_versions) != len(self.subgaits):
                difference = set(self.subgaits.keys()).difference(
                    set(selected_versions.keys())
                )
                output = f"Could not find version for subgaits: {difference}"
            else:
                output = str(selected_versions)
        else:
            output = "Prefix and postfix cannot both be empty"
        self.__getattribute__(f"selectedSubgaits{index}").setText(output)

        self._verify_input()

    def get_subgait_prefix(self, index: int):
        return self.__getattribute__(f"prefix_input{index}").text()

    def get_subgait_postfix(self, index: int):
        return self.__getattribute__(f"postfix_input{index}").text()

    def _clear_subgait_selection(self, index: int):
        self._get_prefix_input_line_edit(index).setText("")
        self._get_postfix_input_line_edit(index).setText("")
        self._get_selected_subgaits_text_browser(index).setText("")

    def _get_selected_subgaits_text_browser(self, index) -> QTextBrowser:
        return self.__getattribute__(f"selectedSubgaits{index}")

    def _get_subgait_button_box(self, index) -> QDialogButtonBox:
        return self.__getattribute__(f"subgaitButtonBox{index}")

    def _get_prefix_input_line_edit(self, index) -> QLineEdit:
        return self.__getattribute__(f"prefix_input{index}")

    def _get_postfix_input_line_edit(self, index) -> QLineEdit:
        return self.__getattribute__(f"postfix_input{index}")

    def _get_parameter_slider(self, index) -> QSlider:
        return self.__getattribute__(f"parameterSlider{index}")

    def _get_parameter_label(self, index) -> QLabel:
        return self.__getattribute__(f"parameterLabel{index}")

    def _get_selected_subgaits(self, index: int) -> dict:
        text = self._get_selected_subgaits_text_browser(index).toPlainText()

        if text == "":
            raise ParseSubgaitException(index)

        try:
            selected_subgaits = ast.literal_eval(text)
        except SyntaxError:
            raise ParseSubgaitException(index)

        if not isinstance(selected_subgaits, dict) or len(selected_subgaits) == 0:
            raise ParseSubgaitException(index)

        return selected_subgaits

    def _reset_all(self):
        for i in range(NUM_SECTIONS):
            self._clear_subgait_selection(i + 1)
        self._init_sliders()
        self.buttonBox.button(QDialogButtonBox.Ok).setEnabled(False)

    def _verify_input(self):
        # Try parsing every necessary subgait selection box
        try:
            self._get_selected_subgaits(1)
            self._get_selected_subgaits(2)

            if self.uses_four_subgait_interpolation:
                self._get_selected_subgaits(3)
                self._get_selected_subgaits(4)

        except ParseError:
            self.buttonBox.button(QDialogButtonBox.Ok).setEnabled(False)
            return

        # Enable the ok button if input is valid
        self.buttonBox.button(QDialogButtonBox.Ok).setEnabled(True)

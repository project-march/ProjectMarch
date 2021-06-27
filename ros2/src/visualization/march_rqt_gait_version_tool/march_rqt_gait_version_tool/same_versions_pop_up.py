from typing import Tuple, Dict

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog
from python_qt_binding import loadUi


class SameVersionsPopUpWindow(QDialog):
    def __init__(self, parent, ui_file):
        super(SameVersionsPopUpWindow, self).__init__(parent=parent, flags=Qt.Window)
        loadUi(ui_file, self)

        self.buttonBox.accepted.connect(self.ok)
        self.buttonBox.rejected.connect(self.cancel)

        # Store the previous pre and postfix per gait
        self._previous_map: Dict[str, Tuple[str, str]] = {}

        self.prefix = ""
        self.postfix = ""
        self.current_gait = ""

    def show_pop_up(self, gait: str):
        """Reset and show pop up."""
        self.prefix = ""
        self.postfix = ""
        self.current_gait = gait

        self.prefix_input.clear()
        self.postfix_input.clear()
        if gait in self._previous_map:
            self.prefix_input.setText(self._previous_map[gait][0])
            self.postfix_input.setText(self._previous_map[gait][1])

        return super(SameVersionsPopUpWindow, self).exec_()

    def cancel(self):
        """Close without applying the values."""
        self.reject()

    def ok(self):
        """Save value while closing."""
        self.prefix = self.prefix_input.text()
        self.postfix = self.postfix_input.text()

        self._previous_map[self.current_gait] = (self.prefix, self.postfix)
        self.current_gait = ""

        self.accept()

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
        self.parameterSlider.valueChanged.connect(self.value_changed)

    def show_pop_up(self, versions):
        """Reset and show pop up."""
        self.baseVersionComboBox.clear()
        self.baseVersionComboBox.addItems(versions)
        self.otherVersionComboBox.clear()
        self.otherVersionComboBox.addItems(versions)
        self.parameterSlider.setValue(50)
        self.parameterLabel.setText("parameter = 0.50")

        self.base_version = ""
        self.other_version = ""
        self.parameter = 0.0
        return super(ParametricPopUpWindow, self).exec_()

    def value_changed(self):
        """Puts the new slider value in the label next to it."""
        self.parameterLabel.setText(
            "parameter = {val:.2f}".format(val=self.parameterSlider.value() / 100.0)
        )

    def cancel(self):
        """Close without applying the values."""
        self.reject()

    def save(self):
        """Check and save value while closing, close if successful."""
        self.base_version = self.baseVersionComboBox.currentText()
        self.other_version = self.otherVersionComboBox.currentText()
        self.parameter = self.parameterSlider.value() / 100.0
        self.accept()


from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QComboBox, QDialog, QDialogButtonBox, QFormLayout, QGridLayout, QLabel, QLineEdit,\
    QMessageBox


class ParametricPopUpWindow(QDialog):
    def __init__(self, parent, width=500, height=200):
        """Base class for creating a pop up window over an existing widget.

        :param parent:
            The parent widget to connect to the pop up
        :param width:
            Starting width of the the pop up widget
        :param height:
            Starting height of the the pop up widget
        """
        super(ParametricPopUpWindow, self).__init__(parent=parent, flags=Qt.Window)
        self.resize(width, height)

        self._layout = QGridLayout(self)
        self.setWindowTitle('parametric gait selector')
        self._form_layout = QFormLayout()

        self._parameter_line_edit = QLineEdit(self)
        self._layout.addWidget(self._parameter_line_edit)
        self._parameter_label = QLabel('parameter', self)
        self._form_layout.addRow(self._parameter_label, self._parameter_line_edit)
        self._layout.addItem(self._form_layout)
        self._button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Close, self)
        self._layout.addWidget(self._button_box)
        self._button_box.accepted.connect(self.save)
        self._button_box.rejected.connect(self.cancel)

        self._button_box.centerButtons()
        base_subgait_label = QLabel('base subgait', self)
        self._base_subgait_menu = QComboBox(self)
        self._base_subgait_menu.currentIndexChanged.connect(self.base_subgait_changed)
        self._form_layout.addRow(base_subgait_label, self._base_subgait_menu)

        other_subgait_label = QLabel('other subgait', self)
        self._other_subgait_menu = QComboBox(self)
        self._other_subgait_menu.currentIndexChanged.connect(self.base_subgait_changed)
        self._form_layout.addRow(other_subgait_label, self._other_subgait_menu)
        self._form_layout.setHorizontalSpacing(80)

    def show_pop_up(self, versions):
        """Add message to the pop up and show the window."""
        self._base_subgait_menu.clear()
        self._base_subgait_menu.addItems(versions)
        self._other_subgait_menu.clear()
        self._other_subgait_menu.addItems(versions)

        self._parameter_line_edit.setText('')

        self.base_version = ''
        self.other_version = ''
        self.parameter = 0.0
        return super(ParametricPopUpWindow, self).exec_()

    def cancel(self):
        self.reject()

    def save(self):
        self.base_version = self._base_subgait_menu.currentText()
        self.other_version = self._other_subgait_menu.currentText()
        try:
            parameter = float(self._parameter_line_edit.text())
        except ValueError:
            QMessageBox.warning(self, '', 'Enter a string that can be converted to a float')
            return
        if 0.0 <= parameter <= 1.0:
            self.parameter = parameter
        else:
            QMessageBox.warning(self, '', 'Enter a float greater or equal than zero and smaller or equal to one.')
            return
        self.accept()

    def base_subgait_changed(self):
        pass

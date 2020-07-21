
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QDialog, QComboBox, QDialogButtonBox, QGridLayout, QFormLayout, QLabel, QLineEdit,QPushButton, QScrollArea, QSizePolicy, QWidget


class ParametricPopUpWindow(QDialog):
    def __init__(self, parent, width=500, height=600):
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
        # self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)

        self._layout = QGridLayout(self)

        # self._scroll_area = QScrollArea()
        # self._scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        # self._scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        # self._scroll_area.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        # self._scroll_area.setWidgetResizable(True)
        #
        # self._content_frame = QWidget(self._scroll_area, flags=Qt.Window)
        # self._content_frame.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        # self._content = QGridLayout(self._content_frame)
        #
        # self.msg_label = QLabel(self)
        # self._content.addWidget(self.msg_label)
        #
        # self._scroll_area.setWidget(self._content_frame)
        # self._layout.addWidget(self._scroll_area)
        self.setWindowTitle('test')
        self._form_layout = QFormLayout()
        self._parameter_line_edit = QLineEdit('parameter here please', self)
        self._layout.addWidget(self._parameter_line_edit)
        self._layout.addItem(self._form_layout)
        self._apply_button = QDialogButtonBox.Ok
        self._cancel_button = QDialogButtonBox.Close
        self._button_box = QDialogButtonBox(self)
        self._layout.addWidget(self._button_box)
        self._button_box.addButton(self._apply_button)
        self._button_box.addButton(self._cancel_button)
        self._button_box.accepted.connect(self.apply)
        self._button_box.rejected.connect(self.cancel)

        self._button_box.centerButtons()
        versions = ['1', '2', '3']
        base_subgait_label = QLabel('base subgait', self)

        self._base_subgait_menu = QComboBox(self)
        self._base_subgait_menu.currentIndexChanged.connect(self.base_subgait_changed)
        self._base_subgait_menu.addItems(versions)

        other_subgait_label = QLabel('other subgait', self)

        self._other_subgait_menu = QComboBox(self)
        self._other_subgait_menu.currentIndexChanged.connect(self.base_subgait_changed)
        self._other_subgait_menu.addItems(versions)

        # self._layout.addItem(base_subgait_label, 1, 1)
        self._form_layout.addRow(base_subgait_label, self._base_subgait_menu)

        self._form_layout.addRow(other_subgait_label, self._other_subgait_menu)

        self.base_version = ''
        self.other_version = ''
        self.parameter = 0.0


    def show_pop_up(self, versions):
        """Add message to the pop up and show the window."""
        self._base_subgait_menu.clear()
        self._base_subgait_menu.addItems(versions)
        self._other_subgait_menu.clear()
        self._other_subgait_menu.addItems(versions)
        return super(ParametricPopUpWindow, self).exec_()

    def cancel(self):
        print('cancel')
        self.accept()

    def apply(self):
        print('accepted')
        base_version = self._base_subgait_menu.currentText()
        if base_version == '':
            self.reject()
        else:
            self.base_version = base_version
        other_version = self._other_subgait_menu.currentText()
        if other_version == '':
            self.reject()
        else:
            self.other_version = other_version
        parameter = float(self._parameter_line_edit.text())
        if 0.0 <= parameter <= 1.0:
            self.parameter = parameter
        else:
            self.reject()

        print('leaving with, base version {0}, other version {1} and parameter {2}'.format(base_version, other_version, parameter))
        self.accept()

    def base_subgait_changed(self):
        pass

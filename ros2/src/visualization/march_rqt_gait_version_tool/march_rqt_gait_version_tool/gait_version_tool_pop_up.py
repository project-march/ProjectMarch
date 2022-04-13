"""Author: Joris Weeda, MV."""
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QDialog,
    QGridLayout,
    QLabel,
    QScrollArea,
    QSizePolicy,
    QWidget,
)


class PopUpWindow(QDialog):
    """Base class for creating a pop-up window over an existing widget.

    Args:
        parent: The parent widget to connect to the pop-up.
        width (int): Starting width of the pop-up widget.
        height (int): Starting height of the pop-up widget.
    """

    def __init__(self, parent, width=500, height=600):
        super(PopUpWindow, self).__init__(parent=parent, flags=Qt.Window)
        self.resize(width, height)
        self.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)

        self._layout = QGridLayout(self)

        self._scroll_area = QScrollArea()
        self._scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._scroll_area.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self._scroll_area.setWidgetResizable(True)

        self._content_frame = QWidget(self._scroll_area, flags=Qt.Window)
        self._content_frame.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Minimum)
        self._content = QGridLayout(self._content_frame)

        self.msg_label = QLabel(self)
        self._content.addWidget(self.msg_label)

        self._scroll_area.setWidget(self._content_frame)
        self._layout.addWidget(self._scroll_area)

    def show_message(self, message):
        """Add message to the pop-up and show the window."""
        self.msg_label.clear()
        self.msg_label.setText(message)
        return super(PopUpWindow, self).show()

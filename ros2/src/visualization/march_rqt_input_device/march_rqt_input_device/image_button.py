from PyQt5.QtGui import QColor, QPainter, QPixmap
from PyQt5.QtWidgets import QAbstractButton, QPushButton, QStyle
from PySide2.QtCore import QEvent

stylesheet = """
QAbstractButton {
    background-color: white;
    border-style: outset;
    border-width: 1px;
    border-radius: 1px;
    border-color: black;
    font: bold 14px;
}
QAbstractButton:hover {
    background-color: black;
    border-style: outset;
    border-width: 1px;
    border-radius: 1px;
    border-color: black;
    font: bold 14px;
}
QAbstractButton:pressed {
    background-color: grey;
    border-style: outset;
    border-width: 1px;
    border-radius: 1px;
    border-color: black;
    font: bold 14px;
}
"""


class ImageButton(QAbstractButton):
    """
    A class to define the standard button on the input device
    """

    def __init__(self, image_path):
        super(ImageButton, self).__init__()
        self.width = 128
        self.height = 128
        self._pixmap = QPixmap(image_path)
        self.setStyleSheet(stylesheet)

    def paintEvent(self, event):  # noqa: N802
        painter = QPainter(self)
        painter.drawPixmap(5, 5, 118, 118, self._pixmap)
        if not self.isEnabled():
            painter.setOpacity(0.3)

        if event.type == QEvent.Enter:
            painter.setOpacity(0.6)

        if event.type == QEvent.Leave:
            painter.setOpacity(0)


from PyQt5.QtGui import QPainter, QPixmap
from PyQt5.QtWidgets import QPushButton


class ImageButton(QPushButton):
    def __init__(self, image_path):
        super(ImageButton, self).__init__()
        self._pixmap = QPixmap(image_path)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._pixmap)

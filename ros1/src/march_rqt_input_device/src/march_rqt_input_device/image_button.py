from PyQt5.QtGui import QColor, QPainter, QPixmap
from PyQt5.QtWidgets import QAbstractButton


class ImageButton(QAbstractButton):
    def __init__(self, image_path):
        super(ImageButton, self).__init__()
        self._pixmap = QPixmap(image_path)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self._pixmap)
        if not self.isEnabled():
            painter.setOpacity(0.3)
            painter.fillRect(event.rect(), QColor(0, 0, 0))

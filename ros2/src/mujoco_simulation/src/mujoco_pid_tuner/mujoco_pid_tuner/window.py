"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QApplication, QMainWindow, QSlider, QVBoxLayout, QWidget
from PyQt5.QtCore import Qt
import sys

class Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()

        self.setWindowTitle("PyQt5 Slider")
        self.setGeometry(100, 100, 400, 300)

        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(50)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TicksBelow)

        self.slider.valueChanged.connect(self.changed_value)

        self.setCentralWidget(self.slider)

    def set_functions(self, publish_gains) -> None:
        self.publish_gains = publish_gains

    def changed_value(self) -> None:
        print("Slider Value:", self.slider.value())
        kp = [float(self.slider.value())]
        kd = [float(self.slider.value())]
        ki = [float(self.slider.value())]
        self.publish_gains(kp, kd, ki)


"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QMainWindow, QSlider, QVBoxLayout, QWidget, QSpinBox
from PyQt5.QtCore import Qt

class Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Slider and Spin Boxes")
        self.setGeometry(100, 100, 400, 200)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        # Spin Boxes
        self.min_spin_box = QSpinBox()
        self.max_spin_box = QSpinBox()
        self.min_spin_box.setMinimum(-1000)
        self.min_spin_box.setMaximum(1000)
        self.max_spin_box.setMinimum(-1000)
        self.max_spin_box.setMaximum(1000)

        self.min_spin_box.valueChanged.connect(self.update_slider_range)
        self.max_spin_box.valueChanged.connect(self.update_slider_range)

        self.layout.addWidget(self.min_spin_box)
        self.layout.addWidget(self.max_spin_box)

        # Slider
        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setMinimum(-1000)
        self.slider.setMaximum(1000)
        self.slider.setValue(0)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self.changed_value)
        self.layout.addWidget(self.slider)

        # Spin Box to change the slider value
        self.spin_box = QSpinBox()
        self.spin_box.setMinimum(-1000)
        self.spin_box.setMaximum(1000)
        self.spin_box.valueChanged.connect(self.update_slider_value)
        self.layout.addWidget(self.spin_box)

    def set_functions(self, publish_gains) -> None:
        self.publish_gains = publish_gains

    def changed_value(self) -> None:
        print("Slider Value:", self.slider.value())
        kp = [float(self.slider.value())]
        kd = [float(self.slider.value())]
        ki = [float(self.slider.value())]
        self.publish_gains(kp, kd, ki)

    def update_slider_range(self):
        min_value = self.min_spin_box.value()
        max_value = self.max_spin_box.value()
        self.slider.setMinimum(min_value)
        self.slider.setMaximum(max_value)

    def update_slider_value(self):
        value = self.spin_box.value()
        self.slider.setValue(value)
"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QMainWindow, QSlider, QWidget, QSpinBox, QGridLayout, QLabel
from PyQt5.QtCore import Qt

class Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Slider and Spin Boxes")
        self.setGeometry(100, 100, 400, 200)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.step_size = 1
        self.actual_min = 0
        self.actual_max = 100
        self.gradient = (self.actual_max - self.actual_min) / self.step_size

        self.layout = QGridLayout()
        self.central_widget.setLayout(self.layout)

        # Spin Boxes
        self.min_spin_box = QSpinBox()
        self.max_spin_box = QSpinBox()
        self.min_spin_box.setMinimum(0)
        self.min_spin_box.setMaximum(1000)
        self.max_spin_box.setMinimum(0)
        self.max_spin_box.setMaximum(1000)
        self.min_spin_box.setValue(0)
        self.max_spin_box.setValue(100)

        self.min_spin_box.valueChanged.connect(self.update_slider_range)
        self.max_spin_box.valueChanged.connect(self.update_slider_range)

        self.min_label = QLabel("Min range")
        self.max_label = QLabel("Max range")

        self.layout.addWidget(self.min_label, 0, 0)
        self.layout.addWidget(self.min_spin_box, 0, 1)
        self.layout.addWidget(self.max_label, 1, 0)
        self.layout.addWidget(self.max_spin_box, 1, 1)

        self.slider = QSlider(Qt.Horizontal, self)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.step_size)
        self.slider.setValue(0)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self.changed_value)
        self.layout.addWidget(self.slider, 2, 0, 1, 2)

        # Spin Box to change the slider value step size
        self.step_spin_box = QSpinBox()
        self.step_spin_box.setMinimum(1)
        self.step_spin_box.setMaximum(10000)
        self.step_spin_box.setValue(self.step_size)
        self.step_spin_box.valueChanged.connect(self.set_step_size)
        self.layout.addWidget(self.step_spin_box, 3, 0, 1, 2)

    def set_functions(self, publish_gains) -> None:
        self.publish_gains = publish_gains

    def changed_value(self) -> None:
        print("Slider Value:", self.slider.value())
        kp = [float(self.slider.value()) * self.gradient] * 8
        kd = [0.0] * 8
        ki = [0.0] * 8
        self.publish_gains(kp, kd, ki)

    def update_slider_range(self):
        self.actual_min = self.min_spin_box.value()
        self.actual_max = self.max_spin_box.value()
        self.gradient = (self.actual_max - self.actual_min) / self.step_size

    def set_step_size(self):
        self.step_size = self.step_spin_box.value()
        self.slider.setMaximum(self.step_size)
        self.gradient = (self.actual_max - self.actual_min) / self.step_size
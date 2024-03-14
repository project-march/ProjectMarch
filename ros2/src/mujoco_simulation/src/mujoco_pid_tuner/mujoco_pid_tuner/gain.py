"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QSlider, QSpinBox, QGridLayout, QLabel
from PyQt5.QtCore import Qt

class Gain:

    def __init__(self, name) -> None:
        self.name = name

        self.step_size = 100
        self.actual_min = 0
        self.actual_max = 20
        self.update_gradient()

    def create_layout(self) -> QGridLayout:
        self.layout = QGridLayout()

        # Name label
        self.name_label = QLabel(self.name)
        self.name_label.setAlignment(Qt.AlignCenter)
        self.name_label.setStyleSheet("font-weight: bold")
        self.name_label.setFixedHeight(60)
        self.layout.addWidget(self.name_label, 0, 0, 1, 2)

        # Min and max range spin boxes
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

        self.layout.addWidget(self.min_label, 1, 0)
        self.layout.addWidget(self.min_spin_box, 1, 1)
        self.layout.addWidget(self.max_label, 2, 0)
        self.layout.addWidget(self.max_spin_box, 2, 1)

        # Slider to change the gain value
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.step_size)
        self.slider.setValue(0)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self.changed_value)
        self.layout.addWidget(self.slider, 3, 0, 1, 2)

        # Spin Box to change the slider value step size
        self.step_label = QLabel("Step size")
        self.layout.addWidget(self.step_label, 4, 0)

        self.step_spin_box = QSpinBox()
        self.step_spin_box.setMinimum(1)
        self.step_spin_box.setMaximum(10000)
        self.step_spin_box.setValue(self.step_size)
        self.step_spin_box.valueChanged.connect(self.set_step_size)
        self.layout.addWidget(self.step_spin_box, 4, 1)

        return self.layout

    def set_callback(self, callback) -> None:
        self.callback = callback

    def set_step_size(self):
        self.step_size = self.step_spin_box.value()
        self.slider.setMaximum(self.step_size)
        self.update_gradient()

    def changed_value(self) -> None:
        new_gain = float(self.slider.value()) * self.gradient
        self.callback(new_gain)

    def update_slider_range(self):
        self.actual_min = self.min_spin_box.value()
        self.actual_max = self.max_spin_box.value()
        self.update_gradient()

    def update_gradient(self):
        self.gradient = (self.actual_max - self.actual_min) / self.step_size
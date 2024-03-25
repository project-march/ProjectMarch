"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QSlider, QSpinBox, QGridLayout, QLabel, QDoubleSpinBox
from PyQt5.QtCore import Qt
import numpy as np

class Gain:

    def __init__(self, name) -> None:
        self.name = name

        self.step_size = 1000
        self.actual_min = 0
        self.actual_max = 25
        self.update_gradient()

        self.gain = 0.0

    def create_layout(self) -> QGridLayout:
        self.layout = QGridLayout()

        # Name label
        self.name_label = QLabel(self.name)
        self.name_label.setAlignment(Qt.AlignCenter)
        self.name_label.setStyleSheet("font-weight: bold")
        self.name_label.setFixedHeight(60)
        self.layout.addWidget(self.name_label, 0, 0, 1, 2)

        # Min and max range spin boxes
        self.min_spin_box = QDoubleSpinBox()
        self.max_spin_box = QDoubleSpinBox()
        self.min_spin_box.setMinimum(0)
        self.max_spin_box.setMinimum(0)
        self.min_spin_box.setValue(self.actual_min)
        self.max_spin_box.setValue(self.actual_max)

        self.min_spin_box.valueChanged.connect(self.update_slider_range)
        self.max_spin_box.valueChanged.connect(self.update_slider_range)

        self.min_label = QLabel("Min range")
        self.max_label = QLabel("Max range")

        self.layout.addWidget(self.min_label, 1, 0)
        self.layout.addWidget(self.min_spin_box, 1, 1)
        self.layout.addWidget(self.max_label, 2, 0)
        self.layout.addWidget(self.max_spin_box, 2, 1)

        # Spin Box to change the slider value step size
        self.step_label = QLabel("Decimal size")
        self.layout.addWidget(self.step_label, 3, 0)

        self.step_spin_box = QSpinBox()
        self.step_spin_box.setMinimum(1)
        self.step_spin_box.setMaximum(100000)
        self.step_spin_box.setValue(self.step_size)
        self.step_spin_box.valueChanged.connect(self.set_step_size)
        self.layout.addWidget(self.step_spin_box, 3, 1)

        # Slider to change the gain value
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(self.step_size)
        self.slider.setValue(0)
        self.slider.setTickInterval(10)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.valueChanged.connect(self.update_gain_value)
        self.layout.addWidget(self.slider, 4, 0, 1, 2)

        # Spin Box to change the slider value directly
        self.slider_label = QLabel("Current gain")
        self.layout.addWidget(self.slider_label, 5, 0)

        self.slider_spin_box = QDoubleSpinBox()
        self.slider_spin_box.setMinimum(0)
        self.slider_spin_box.setMaximum(self.actual_max)
        self.slider_spin_box.setValue(self.gain)
        self.slider_spin_box.valueChanged.connect(self.update_slider_value)
        self.slider_spin_box.setDecimals(np.log10(self.step_size))
        self.layout.addWidget(self.slider_spin_box, 5, 1)

        return self.layout

    def set_callback(self, callback) -> None:
        self.callback = callback

    def set_step_size(self):
        self.step_size = self.step_spin_box.value()
        self.slider_spin_box.setDecimals(np.log10(self.step_size))
        self.slider.setMaximum(self.step_size)
        self.update_gradient()

    def update_gain_value(self) -> None:
        self.gain = float(self.slider.value()) * self.gradient
        self.slider_spin_box.setValue(self.gain)

    def update_slider_range(self) -> None:
        self.actual_min = self.min_spin_box.value()
        self.actual_max = self.max_spin_box.value()
        self.slider_spin_box.setMinimum(self.actual_min)
        self.slider_spin_box.setMaximum(self.actual_max)
        self.update_gradient()

    def update_slider_value(self) -> None:
        slider_value = int(self.slider_spin_box.value() / self.gradient)
        self.slider.setValue(slider_value)

    def update_gradient(self):
        self.gradient = (self.actual_max - self.actual_min) / self.step_size

    def get_gain(self) -> float:
        return self.gain
    
    def set_gain(self, gain) -> None:
        self.gain = gain
        self.update_slider_value()
        self.update_gain_value()
        self.slider_spin_box.setValue(gain)
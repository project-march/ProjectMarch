"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QVBoxLayout, QLabel
from PyQt5.QtCore import Qt

from .gain import Gain

class Joint:
    """
    This class is used to create a joint object that contains the proportional, derivative, and integral gains.

    """

    def __init__(self, name) -> None:
        self.name = name

        self.kp = Gain("Proportional Gains")
        self.kd = Gain("Derivative Gains")
        self.ki = Gain("Integral Gains")

    def create_layout(self) -> QVBoxLayout:
        self.layout = QVBoxLayout()

        # Bold and large label, center and wrap the text. Set the height to the minimum
        self.name_label = QLabel(self.name)
        self.name_label.setAlignment(Qt.AlignCenter)
        self.name_label.setStyleSheet("font-weight: bold")
        self.name_label.setStyleSheet("font-size: 20px")
        self.name_label.setWordWrap(True)
        self.name_label.setFixedHeight(40)
        self.layout.addWidget(self.name_label)

        self.layout.addLayout(self.kp.create_layout())
        self.layout.addLayout(self.kd.create_layout())
        self.layout.addLayout(self.ki.create_layout())

        return self.layout
    
    def get_gains(self):
        gain_p = self.kp.get_gain()
        gain_d = self.kd.get_gain()
        gain_i = self.ki.get_gain()
        return gain_p, gain_d, gain_i
    
    def set_gains(self, kp, kd, ki):
        self.kp.set_gain(kp)
        self.kd.set_gain(kd)
        self.ki.set_gain(ki)
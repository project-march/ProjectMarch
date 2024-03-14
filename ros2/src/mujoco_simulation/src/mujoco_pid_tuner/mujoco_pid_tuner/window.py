"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout

from .joint import Joint

class Window(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Mujoco PID Tuner")
        self.setGeometry(100, 100, 400, 300)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.joints = {
            'HAA': Joint('Hip Abduction/Adduction'),
            'HFE': Joint('Hip Flexion/Extension'),
            'KFE': Joint('Knee Flexion/Extension'),
            'ADPF': Joint('Ankle Dorsiflexion/Plantarflexion'),
        }

    def configure_ui(self) -> None:

        self.layout = QHBoxLayout()
        self.central_widget.setLayout(self.layout)

        # Set the layout for each joint with equal width
        for i, joint in enumerate(self.joints.values()):
            self.layout.addLayout(joint.create_layout())

    # def set_publish(self, publish) -> None:
    #     self.publish = publish


    # def update(self, value) -> None:
    #     self.publish(self.kp_box.slider.value(), self.kd_box.slider.value(), 0)
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
            'HAA': Joint('HAA'),
            'HFE': Joint('HFE'),
            'KFE': Joint('KFE'),
            'ADPF': Joint('ADPF'),
        }

    def configure_ui(self) -> None:

        self.layout = QHBoxLayout()
        self.central_widget.setLayout(self.layout)

        # Set the layout for each joint with equal width
        for _, joint in enumerate(self.joints.values()):
            self.layout.addLayout(joint.create_layout())

    def create_menu(self) -> None:
        """
        TODO:
        - Create a button to set the gains to the default values
        - Create a button and functionality to save the gains to a file
        - Create a button and functionality to load the gains from a file
        - Create a functionality to store the gains as YAML
        """
        pass

    def get_gains(self):
        kp = []
        kd = []
        ki = []

        for joint in self.joints.values():
            p, d, i = joint.get_gains()
            kp.append(p)
            kd.append(d)
            ki.append(i)

        return kp, kd, ki
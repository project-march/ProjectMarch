"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, QLabel, QSlider, QDoubleSpinBox, QHBoxLayout, QPushButton

class Window(QMainWindow):
    
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Inverse Kinematics Testing Environment")
        self.setGeometry(100, 100, 400, 300)

        # Variables
        self.is_publishing_ = False

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        # Create three vertical layouts: one for each foot and one for the menu
        self.left_foot_layout = QVBoxLayout()
        self.right_foot_layout = QVBoxLayout()
        self.menu_layout = QVBoxLayout()

        # Set the layout for the left foot: number text fields for x, y, and z
        # Set labels next to the double spin box horizontally
        self.left_foot_label = QLabel("Left Foot")

        self.left_foot_x_layout = QHBoxLayout()
        self.left_foot_x_label = QLabel("X")
        self.left_foot_x = QDoubleSpinBox()
        self.left_foot_x.setRange(-1.0, 1.0)
        self.left_foot_x.setValue(0.19)
        self.left_foot_x.setSingleStep(0.001)
        self.left_foot_x.setDecimals(4)
        self.left_foot_x.valueChanged.connect(self.update_left_foot_x)
        self.left_foot_x_layout.addWidget(self.left_foot_x_label)
        self.left_foot_x_layout.addWidget(self.left_foot_x)

        self.left_foot_y_layout = QHBoxLayout()
        self.left_foot_y_label = QLabel("Y")
        self.left_foot_y = QDoubleSpinBox()
        self.left_foot_y.setRange(-0.5, 0.5)
        self.left_foot_y.setValue(0.16)
        self.left_foot_y.setSingleStep(0.001)
        self.left_foot_y.setDecimals(4)
        self.left_foot_y.valueChanged.connect(self.update_left_foot_y)
        self.left_foot_y_layout.addWidget(self.left_foot_y_label)
        self.left_foot_y_layout.addWidget(self.left_foot_y)


        self.left_foot_z_layout = QHBoxLayout()
        self.left_foot_z_label = QLabel("Z")
        self.left_foot_z = QDoubleSpinBox()
        self.left_foot_z.setRange(-0.9, -0.0)
        self.left_foot_z.setValue(-0.87)
        self.left_foot_z.setSingleStep(0.001)
        self.left_foot_z.setDecimals(4)
        self.left_foot_z.valueChanged.connect(self.update_left_foot_z)
        self.left_foot_z_layout.addWidget(self.left_foot_z_label)
        self.left_foot_z_layout.addWidget(self.left_foot_z)

        self.left_foot_layout.addWidget(self.left_foot_label)
        self.left_foot_layout.addLayout(self.left_foot_x_layout)
        self.left_foot_layout.addLayout(self.left_foot_y_layout)
        self.left_foot_layout.addLayout(self.left_foot_z_layout)

        # Set the layout for the right foot: number text fields for x, y, and z
        # Set labels next to the double spin box horizontally
        self.right_foot_label = QLabel("Right Foot")

        self.right_foot_x_layout = QHBoxLayout()
        self.right_foot_x_label = QLabel("X")
        self.right_foot_x = QDoubleSpinBox()
        self.right_foot_x.setRange(-1.0, 1.0)
        self.right_foot_x.setValue(0.19)
        self.right_foot_x.setSingleStep(0.001)
        self.right_foot_x.setDecimals(4)
        self.right_foot_x.valueChanged.connect(self.update_right_foot_x)
        self.right_foot_x_layout.addWidget(self.right_foot_x_label)
        self.right_foot_x_layout.addWidget(self.right_foot_x)

        self.right_foot_y_layout = QHBoxLayout()
        self.right_foot_y_label = QLabel("Y")
        self.right_foot_y = QDoubleSpinBox()
        self.right_foot_y.setRange(-0.5, 0.5)
        self.right_foot_y.setValue(-0.16)
        self.right_foot_y.setSingleStep(0.001)
        self.right_foot_y.setDecimals(4)
        self.right_foot_y.valueChanged.connect(self.update_right_foot_y)
        self.right_foot_y_layout.addWidget(self.right_foot_y_label)
        self.right_foot_y_layout.addWidget(self.right_foot_y)

        self.right_foot_z_layout = QHBoxLayout()
        self.right_foot_z_label = QLabel("Z")
        self.right_foot_z = QDoubleSpinBox()
        self.right_foot_z.setRange(-0.9, -0.0)
        self.right_foot_z.setValue(-0.87)
        self.right_foot_z.setSingleStep(0.001)
        self.right_foot_z.setDecimals(4)
        self.right_foot_z.valueChanged.connect(self.update_right_foot_z)
        self.right_foot_z_layout.addWidget(self.right_foot_z_label)
        self.right_foot_z_layout.addWidget(self.right_foot_z)

        self.right_foot_layout.addWidget(self.right_foot_label)
        self.right_foot_layout.addLayout(self.right_foot_x_layout)
        self.right_foot_layout.addLayout(self.right_foot_y_layout)
        self.right_foot_layout.addLayout(self.right_foot_z_layout)

        # Set the layout for the menu: buttons to send the feet positions and stop the command
        self.send_button = QPushButton('Publish')
        self.send_button.clicked.connect(self.publish_ik_command)

        self.stop_button = QPushButton('Stop')
        self.stop_button.clicked.connect(self.stop_ik_command)

        self.menu_layout.addWidget(self.send_button)
        self.menu_layout.addWidget(self.stop_button)

        # Set the central widget layout
        self.layout = QHBoxLayout()
        self.central_widget.setLayout(self.layout)
        self.layout.addLayout(self.left_foot_layout)
        self.layout.addLayout(self.right_foot_layout)
        self.layout.addLayout(self.menu_layout)

    def update_left_foot_x(self, value):
        self.left_foot_x.value = value

    def update_left_foot_y(self, value):
        self.left_foot_y.value = value

    def update_left_foot_z(self, value):
        self.left_foot_z.value = value

    def update_right_foot_x(self, value):
        self.right_foot_x.value = value

    def update_right_foot_y(self, value):
        self.right_foot_y.value = value

    def update_right_foot_z(self, value):
        self.right_foot_z.value = value

    def publish_ik_command(self):
        self.is_publishing_ = True
        self.left_foot_x.setDisabled(True)
        self.left_foot_y.setDisabled(True)
        self.left_foot_z.setDisabled(True)
        self.right_foot_x.setDisabled(True)
        self.right_foot_y.setDisabled(True)
        self.right_foot_z.setDisabled(True)

    def stop_ik_command(self):
        self.is_publishing_ = False
        self.left_foot_x.setDisabled(False)
        self.left_foot_y.setDisabled(False)
        self.left_foot_z.setDisabled(False)
        self.right_foot_x.setDisabled(False)
        self.right_foot_y.setDisabled(False)
        self.right_foot_z.setDisabled(False)
import pyqtgraph as pg
import numpy as np
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QSlider, QWidget, QGridLayout, QPushButton
from march_goniometric_ik_solver.ik_solver import Pose
import copy

DEFAULT_HIP_FRACTION = 0.5
DEFAULT_KNEE_BEND = np.deg2rad(8)
REDUCE_DF_REAR = False
REDUCE_DF_FRONT = False

X_MIN = 0.0
X_MAX = 0.6
Y_MIN = -0.3
Y_MAX = 0.3

JOINT_NAMES = [
    "ankle1",
    "hip1_aa",
    "hip1_fe",
    "knee1",
    "ankle2",
    "hip2_aa",
    "hip2_fe",
    "knee2",
]


class LiveWidget:
    def __init__(self) -> None:
        self.slider_x = 0
        self.slider_y = 0

        self.create_window()
        self.create_plot()
        self.create_sliders()
        self.create_buttons()
        self.create_table()
        self.fill_layout()

    def create_window(self):
        self.window = QWidget()
        self.window.setWindowTitle("Test")
        self.layout = QGridLayout(self.window)
        pg.setConfigOptions(antialias=True)

    def create_plot(self):
        self.plot_window = pg.GraphicsWindow()
        self.plot_window.setBackground("w")
        plot = self.plot_window.addPlot()
        plot.setAspectLocked()
        plot.showGrid(x=True, y=True)

        self.exo = plot.plot(pen="k", symbol="o", symbolSize=6)
        self.pose = Pose()
        self.update_pose()

    def create_sliders(self):
        self.slider_horizontal = QSlider()
        self.slider_horizontal.setOrientation(Qt.Horizontal)
        self.slider_horizontal.valueChanged.connect(self.update_x_slider)

        self.slider_vertical = QSlider()
        self.slider_vertical.setValue(50)
        self.slider_vertical.setOrientation(Qt.Vertical)
        self.slider_vertical.valueChanged.connect(self.update_y_slider)

    def create_buttons(self):
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset)

    def create_table(self):
        self.tables = QGridLayout()
        self.table = pg.TableWidget()
        self.table.horizontalHeader().hide()
        self.update_table()
        self.tables.addWidget(self.table, 0, 0)

    def fill_layout(self):
        self.layout.addWidget(self.slider_vertical, 0, 0)
        self.layout.addWidget(self.plot_window, 0, 1)
        self.layout.addWidget(self.slider_horizontal, 1, 1)
        self.layout.addLayout(self.tables, 0, 2)
        self.layout.addWidget(self.reset_button, 1, 2)

    def update_x_slider(self, value):
        self.slider_x = (value / 99) * (X_MAX - X_MIN) + X_MIN
        self.update_pose()
        self.update_table()

    def update_y_slider(self, value):
        self.slider_y = (value / 99) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose()
        self.update_table()

    def reset(self):
        self.slider_horizontal.setValue(0)
        self.slider_vertical.setValue(50)
        self.slider_x = 0
        self.slider_y = 0
        self.update_pose()
        self.update_table()

    def update_pose(self):
        self.pose.solve_end_position(
            self.slider_x,
            self.slider_y,
            0.0,
            "",
            DEFAULT_HIP_FRACTION,
            DEFAULT_KNEE_BEND,
            REDUCE_DF_FRONT,
            REDUCE_DF_REAR,
        )
        positions = self.pose.calculate_joint_positions()
        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]
        self.exo.setData(x=positions_x, y=positions_y)

    def update_table(self):
        joint_angles = self.pose.pose_left
        joint_angles_degrees = [np.rad2deg(angle) for angle in joint_angles]

        data = {}
        for joint, angle_rad, angle_deg in zip(
            JOINT_NAMES, np.round(joint_angles, 3), np.round(joint_angles_degrees, 3)
        ):
            data[joint] = [angle_rad, angle_deg]

        self.table.setData(data)

    def show(self):
        self.window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    live_widget = LiveWidget()
    live_widget.show()
    sys.exit(app.exec_())

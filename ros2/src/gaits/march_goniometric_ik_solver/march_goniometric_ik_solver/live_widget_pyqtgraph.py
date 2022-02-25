import pyqtgraph as pg
import numpy as np
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QSlider, QWidget, QGridLayout, QPushButton

from march_goniometric_ik_solver.ik_solver import Pose

DEFAULT_HIP_FRACTION = 0.5
DEFAULT_KNEE_BEND = np.deg2rad(8)
REDUCE_DF_REAR = False
REDUCE_DF_FRONT = False

X_MIN = 0.0
X_MAX = 0.6
Y_MIN = -0.3
Y_MAX = 0.3


class LiveWidget:
    def __init__(self) -> None:
        self.slider_x = 0
        self.slider_y = 0

        self.create_window()
        self.create_plot()
        self.create_sliders()
        self.create_buttons()
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

        self.exo = plot.plot(pen="k")
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

    def fill_layout(self):
        self.layout.addWidget(self.slider_vertical, 0, 0)
        self.layout.addWidget(self.plot_window, 0, 1)
        self.layout.addWidget(self.slider_horizontal, 1, 1)
        self.layout.addWidget(self.reset_button, 0, 2)

    def update_x_slider(self, value):
        self.slider_x = (value / 99) * (X_MAX - X_MIN) + X_MIN
        self.update_pose()

    def update_y_slider(self, value):
        self.slider_y = (value / 99) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose()

    def reset(self):
        self.slider_horizontal.setValue(0)
        self.slider_vertical.setValue(50)
        self.slider_x = 0
        self.slider_y = 0
        self.update_pose()

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

    def show(self):
        self.window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    live_widget = LiveWidget()
    live_widget.show()
    sys.exit(app.exec_())

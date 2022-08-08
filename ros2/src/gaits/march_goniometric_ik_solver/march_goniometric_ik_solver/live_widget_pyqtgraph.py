"""Author: Jelmer de Wolde, MVII."""

from typing import List
import pyqtgraph as pg
import numpy as np
import copy
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QSlider, QWidget, QGridLayout, QPushButton, QCheckBox
from march_goniometric_ik_solver.ik_solver import Pose, LENGTH_HIP, JOINT_NAMES
from march_goniometric_ik_solver.ik_solver_parameters import IKSolverParameters
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError
from march_gait_selection.dynamic_interpolation.gaits.dynamic_joint_trajectory import DynamicJointTrajectory
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration

X_MIN = 0.0
X_MAX = 0.7
Y_MIN = -0.35
Y_MAX = 0.35

MIDPOINT_HEIGHT = 0.15
TRAJECTORY_SAMPLES = 99
IK_SOLVER_PARAMTERS = IKSolverParameters(dorsiflexion_at_end_position=0.0)
DURATION = 2.0
NUMBER_OF_JOINTS = 8


class LiveWidget:
    """A widget to easily check the solutions of the IK solver for a given x,y location of the ankle.

    This widget has been made for debugging purposes, to evaluate poses the IK solver provides as solution for a given goal location. This widget can be executed by sourcing ROS2, March ROS2 and running this script with python: sfox && sros2 && python3 live_widget_pyqtgraph.py

    Attributes:
        default_hip_fraction (float): the default fraction between the two feet (forward) at which the hip is desired.
        default_knee_bend (float): the default bending of the knee for a straight leg.
    """

    def __init__(self) -> None:
        self.sliders = {
            "last": {"x": 0.0, "y": 0.0},
            "next": {"x": 0.0, "y": 0.0},
            "mid": {"fraction": 0.0, "deviation": 0.0, "height": MIDPOINT_HEIGHT},
        }
        self.joint_trajectories: List[DynamicJointTrajectory] = []
        self.reduce_df_front = True

        self.create_window()
        self.create_plot()
        self.create_sliders()
        self.create_buttons()
        self.create_table()
        self.fill_layout()

    @property
    def midpoint_height(self):
        """Returns the midpoint hegiht based on foot locations and midpoint height slider."""
        return self.sliders["mid"]["height"] + max(-self.sliders["last"]["y"], self.sliders["next"]["y"])

    def create_window(self):
        """Creates a QT window."""
        self.window = QWidget()
        self.window.setWindowTitle("IK Solver - Widget")
        self.layout = QGridLayout(self.window)
        pg.setConfigOptions(antialias=True)

    def create_plot(self):
        """Creates a plot where we can visualize a pose."""
        self.plot_window = pg.GraphicsWindow()
        self.plot_window.setBackground("w")
        plot = self.plot_window.addPlot()
        plot.setAspectLocked()
        plot.showGrid(x=True, y=True)

        self.poses = {
            "last": Pose(IK_SOLVER_PARAMTERS),
            "mid_pre": Pose(IK_SOLVER_PARAMTERS),
            "mid_post": Pose(IK_SOLVER_PARAMTERS),
            "next": Pose(IK_SOLVER_PARAMTERS),
        }
        self.plots = {
            "mid": plot.plot(pen="g", symbol="o", symbolSize=6),
            "mid_pre": plot.plot(pen=0.8, symbol="o", symbolSize=6),
            "mid_post": plot.plot(pen=0.8, symbol="o", symbolSize=6),
            "last": plot.plot(pen="b", symbol="o", symbolSize=6),
            "next": plot.plot(pen="k", symbol="o", symbolSize=6),
            "mid_of_step": plot.plot(symbolPen="r", symbol="o", symbolSize=6),
            "mid_pre_ankle": plot.plot(symbolPen="b", symbol="o", symbolSize=6),
            "mid_post_ankle": plot.plot(symbolPen="k", symbol="o", symbolSize=6),
        }
        self.update_poses()

    def create_sliders(self):
        """Creates sliders to control the x and y positions of the poses."""
        self.slider_last_x = QSlider()
        self.slider_last_x.setOrientation(Qt.Horizontal)
        self.slider_last_x.setValue(99)
        self.slider_last_x.valueChanged.connect(self.update_last_x)

        self.slider_next_x = QSlider()
        self.slider_next_x.setOrientation(Qt.Horizontal)
        self.slider_next_x.valueChanged.connect(self.update_next_x)

        self.slider_midpoint_fraction = QSlider()
        self.slider_midpoint_fraction.setOrientation(Qt.Horizontal)
        self.slider_midpoint_fraction.valueChanged.connect(self.update_midpoint_fraction)

        self.slider_midpoint_deviation = QSlider()
        self.slider_midpoint_deviation.setOrientation(Qt.Horizontal)
        self.slider_midpoint_deviation.valueChanged.connect(self.update_midpoint_deviation)

        self.slider_midpoint_height = QSlider()
        self.slider_midpoint_height.setOrientation(Qt.Horizontal)
        self.slider_midpoint_height.setValue(99)
        self.slider_midpoint_height.valueChanged.connect(self.update_midpoint_height)

        self.horizontal_sliders = QGridLayout()
        self.horizontal_sliders_top = QGridLayout()
        self.horizontal_sliders_bottom = QGridLayout()

        self.horizontal_sliders_top.addWidget(self.slider_last_x, 0, 0)
        self.horizontal_sliders_top.addWidget(self.slider_next_x, 0, 1)

        self.horizontal_sliders_bottom.addWidget(self.slider_midpoint_deviation, 0, 0)
        self.horizontal_sliders_bottom.addWidget(self.slider_midpoint_height, 0, 1)

        self.horizontal_sliders.addLayout(self.horizontal_sliders_top, 0, 0)
        self.horizontal_sliders.addWidget(self.slider_midpoint_fraction, 1, 0)
        self.horizontal_sliders.addLayout(self.horizontal_sliders_bottom, 2, 0)

        self.slider_last_y = QSlider()
        self.slider_last_y.setValue(50)
        self.slider_last_y.setOrientation(Qt.Vertical)
        self.slider_last_y.valueChanged.connect(self.update_last_y)

        self.slider_next_y = QSlider()
        self.slider_next_y.setValue(50)
        self.slider_next_y.setOrientation(Qt.Vertical)
        self.slider_next_y.valueChanged.connect(self.update_next_y)

        self.vertical_sliders = QGridLayout()
        self.vertical_sliders.addWidget(self.slider_last_y, 0, 0)
        self.vertical_sliders.addWidget(self.slider_next_y, 0, 1)

    def create_buttons(self):
        """Create buttons to reset pose and turn dorsiflexion reduction on or off."""
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset)

        self.df_front_button = QCheckBox("DF front")
        self.df_front_button.setChecked(True)
        self.df_front_button.clicked.connect(self.toggle_df_front)

        self.buttons = QGridLayout()
        self.buttons.addWidget(self.reset_button, 0, 0)
        self.buttons.addWidget(self.df_front_button, 0, 1)

    def create_table(self):
        """Create a table where we write the angles of all joints."""
        self.table = QGridLayout()
        self.tables = {"last": pg.TableWidget(), "next": pg.TableWidget()}
        self.update_tables()
        for pose in ["last", "next"]:
            self.table.addWidget(self.tables[pose], list(self.tables.keys()).index(pose), 0)

    def fill_layout(self):
        """Fill the layout of the window with all the created elements."""
        self.layout.addLayout(self.vertical_sliders, 0, 0)
        self.layout.addWidget(self.plot_window, 0, 1)
        self.layout.addLayout(self.horizontal_sliders, 1, 1)
        self.layout.addLayout(self.table, 0, 2)
        self.layout.addLayout(self.buttons, 1, 2)

    def update_last_x(self, value):
        """Update the x value of last pose."""
        self.sliders["last"]["x"] = (1 - (value / 99)) * (X_MAX - X_MIN) + X_MIN
        self.update_pose("last")
        self.reset_midpoint_fraction()
        self.update_pose("mid_pre")
        self.update_pose("mid_post")
        self.update_tables()

    def update_next_x(self, value):
        """Update the x value of next pose."""
        self.sliders["next"]["x"] = (value / 99) * (X_MAX - X_MIN) + X_MIN
        self.update_pose("next")
        self.reset_midpoint_fraction()
        self.update_pose("mid_pre")
        self.update_pose("mid_post")
        self.update_tables()

    def update_last_y(self, value):
        """Update the y value of last pose."""
        self.sliders["last"]["y"] = (1 - (value / 99)) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose("last")
        self.reset_midpoint_fraction()
        self.update_pose("mid_pre")
        self.update_pose("mid_post")
        self.update_tables()

    def update_next_y(self, value):
        """Update the y value of next pose."""
        self.sliders["next"]["y"] = (value / 99) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose("next")
        self.reset_midpoint_fraction()
        self.update_pose("mid_pre")
        self.update_pose("mid_post")
        self.update_tables()

    def update_midpoint_fraction(self, value):
        """Update the fraction of the step at which we want to show a midpoint."""
        self.sliders["mid"]["fraction"] = value / 100
        if self.increasing_setpoint_times():
            self.plot_pose("mid", self.pose_from_dynamic_joint_trajectory())

    def update_midpoint_deviation(self, value):
        """Update the deviation used to create the midpoints."""
        self.sliders["mid"]["deviation"] = value / 100
        self.update_pose("mid_pre")
        self.update_pose("mid_post")

    def update_midpoint_height(self, value):
        """Update the height used to create the midpoints."""
        self.sliders["mid"]["height"] = value / 100 * MIDPOINT_HEIGHT
        self.update_pose("mid_pre")
        self.update_pose("mid_post")

    def reset(self):
        """Reset to default pose."""
        self.slider_last_x.setValue(99)
        self.slider_next_x.setValue(0)
        self.slider_last_y.setValue(50)
        self.slider_next_y.setValue(50)
        for pose in ["last", "next"]:
            for axis in ["x", "y"]:
                self.sliders[pose][axis] = 0
        self.update_poses()
        self.reset_midpoint_poses()
        self.update_tables()

    def reset_midpoint_fraction(self):
        """Resets the slider and clears the plot."""
        self.slider_midpoint_fraction.setValue(0)
        self.plots["mid"].setData(x=[], y=[])

    def reset_midpoint_poses(self):
        """Clears the plots for the midpoint poses."""
        for pose_name in ["mid_pre", "mid_post"]:
            self.plots[pose_name].setData(x=[], y=[])

    def toggle_df_front(self):
        """Toggle dorsiflexion reduction of front lef."""
        self.reduce_df_front = not self.reduce_df_front
        self.update_poses()
        self.update_tables()

    def get_midpoint_fraction_based_on_deviation(self, pose: str):
        """Get the midpoint fraction based on the deviation."""
        if pose == "mid_pre":
            return 0.5 * (1 - self.sliders["mid"]["deviation"])
        elif pose == "mid_post":
            return 0.5 * (1 + self.sliders["mid"]["deviation"])
        else:
            return 0.5

    def setpoint_time(self, pose_name: str):
        """Returns the setpoint time for the given pose."""
        if pose_name == "last":
            return 0.0
        elif pose_name == "next":
            return DURATION
        else:
            return DURATION * self.get_midpoint_fraction_based_on_deviation(pose_name)

    def increasing_setpoint_times(self):
        """Returns true if the setpoint times for all succesive poses are increasing."""
        is_increasing = 1
        for n in range(1, len(self.poses)):
            if self.setpoint_time(list(self.poses.keys())[n]) > self.setpoint_time(list(self.poses.keys())[n - 1]):
                is_increasing += 1
        return is_increasing == len(self.poses)

    def interpolate_trough_points(self):
        """Interpolates setpoints to joint trajectory for every joint."""
        if self.increasing_setpoint_times():
            self.joint_trajectories = []
            for n in range(NUMBER_OF_JOINTS):
                setpoint_list = []
                for pose_name in self.poses.keys():
                    time = Duration(self.setpoint_time(pose_name))
                    position = (
                        self.poses[pose_name].pose_left[n]
                        if pose_name == "last"
                        else self.poses[pose_name].pose_right[n]
                    )
                    setpoint_list.append(Setpoint(time, position, 0.0))
                self.joint_trajectories.append(DynamicJointTrajectory(setpoint_list))

    def pose_from_dynamic_joint_trajectory(self):
        """Creates a pose from all joints dynamic joint trajectories, on time given by the fraction slider."""
        pose_list = []
        for n in range(NUMBER_OF_JOINTS):
            pose_list.append(
                self.joint_trajectories[n]
                .get_interpolated_setpoint(self.sliders["mid"]["fraction"] * DURATION)
                .position
            )
        return Pose(IK_SOLVER_PARAMTERS, pose_list)

    def update_pose(self, pose_name: str):
        """Update the given pose."""
        try:
            if pose_name in ["mid_pre", "mid_post"]:
                fraction = self.get_midpoint_fraction_based_on_deviation(pose_name)
                self.poses[pose_name] = copy.deepcopy(self.poses["last"])
                self.poses[pose_name].solve_mid_position(self.poses["next"], fraction, self.midpoint_height, "")
            else:
                self.poses[pose_name].solve_end_position(
                    self.sliders[pose_name]["x"],
                    self.sliders[pose_name]["y"],
                    LENGTH_HIP,
                    "",
                )
        except (PositionSoftLimitError) as value_error:
            print(value_error)

        self.plot_pose(pose_name, self.poses[pose_name])

    def plot_pose(self, pose_name: str, pose: Pose):
        """Plots the given pose."""
        positions = list(pose.calculate_joint_positions().values())

        # shift positions to have toes of stand leg at (0,0):
        if pose_name == "last":
            positions = [pos - positions[-1] for pos in positions]
        else:
            positions = [pos - positions[0] for pos in positions]

        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]
        self.plots[pose_name].setData(x=positions_x, y=positions_y)
        self.update_mid_of_step()
        self.update_midpoint_ankle_postions()
        self.interpolate_trough_points()

    def update_poses(self):
        """Update all poses."""
        for pose in ["last", "next"]:
            self.update_pose(pose)

    def update_mid_of_step(self):
        """Update the point that is in the middle of the step."""
        step_size_last = self.poses["last"].pos_ankle2[0]
        step_size_next = self.poses["next"].pos_ankle2[0]
        mid_of_step = (-step_size_last + step_size_next) / 2
        mid_of_step -= self.poses["next"].pos_toes1[0]  # Shift since we center around toes
        self.plots["mid_of_step"].setData(x=[mid_of_step], y=[0])

    def update_midpoint_ankle_postions(self):
        """Update the two points at which the ankle is desired for the two midpoints."""
        for pose in ["mid_pre", "mid_post"]:
            fraction = self.get_midpoint_fraction_based_on_deviation(pose)
            x = self.poses["last"].get_ankle_mid_x(fraction, self.poses["next"])
            x -= self.poses["next"].pos_toes1[0]  # Shift since we center around toes.
            y = self.midpoint_height
            self.plots[pose + "_ankle"].setData(x=[x], y=[y])

    def update_tables(self):
        """Update the tables."""
        for pose in ["last", "next"]:
            joint_angles = self.poses[pose].pose_left
            joint_angles_degrees = [np.rad2deg(angle) for angle in joint_angles]

            data = []
            for joint, angle_rad, angle_deg in zip(
                JOINT_NAMES,
                np.round(joint_angles, 3),
                np.round(joint_angles_degrees, 3),
            ):
                data.append([joint, angle_rad, angle_deg])

            self.tables[pose].setData(data)
            self.tables[pose].setHorizontalHeaderLabels(["", "rad", "deg"])
            self.tables[pose].verticalHeader().hide()

    def show(self):
        """Show the tool."""
        self.window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    live_widget = LiveWidget()
    live_widget.show()
    sys.exit(app.exec_())

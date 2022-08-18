"""Author: Jelmer de Wolde, MVII."""

from typing import List
import pyqtgraph as pg
import numpy as np
import copy
import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QSlider, QWidget, QGridLayout, QPushButton, QCheckBox
from march_goniometric_ik_solver.ik_solver import Pose, LENGTH_HIP, JOINT_NAMES, check_on_limits
from march_goniometric_ik_solver.ik_solver_parameters import IKSolverParameters
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError
from march_gait_selection.dynamic_interpolation.gaits.dynamic_joint_trajectory import DynamicJointTrajectory
from march_gait_preprocessor.gait_preprocessor import GaitPreprocessor
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration

X_MIN = 0.0
X_MAX = 0.7
Y_MIN = -0.35
Y_MAX = 0.35

MIDPOINT_HEIGHT = 0.15
IK_SOLVER_PARAMETERS = IKSolverParameters(dorsiflexion_at_end_position=0.0)
DEFAULT_DURATION = 1.3
NUMBER_OF_JOINTS = 8

MIN_MIDPOINT_FRACTION = 0.1
MAX_MIDPOINT_FRACTION = 0.99


class LiveWidget:
    """A widget to easily check the solutions of the IK solver for a given x,y location of the ankle.

    This widget has been made for debugging purposes, to evaluate poses the IK solver provides as solution for a given goal location.
    This widget can be executed by sourcing ROS2, March ROS2 and running this script with python: sfox && sros2 && python3 live_widget_pyqtgraph.py
    """

    def __init__(self) -> None:
        self.sliders = {
            "last": {"x": 0.0, "y": 0.0},
            "next": {"x": 0.0, "y": 0.0},
            "mid": {"fraction": 0.0, "deviation": 0.0, "shift": 0.0, "height": MIDPOINT_HEIGHT},
        }
        self.poses = {
            "last": Pose(IK_SOLVER_PARAMETERS),
            "mid_pre": Pose(IK_SOLVER_PARAMETERS),
            "mid_post": Pose(IK_SOLVER_PARAMETERS),
            "next": Pose(IK_SOLVER_PARAMETERS),
        }
        self.pose_colours = {
            "mid": "g",
            "mid_pre": 0.8,
            "mid_post": 0.8,
            "last": "b",
            "next": "k",
        }
        self.joint_trajectories: List[DynamicJointTrajectory] = []
        self.show_midpoints = True

        self.create_window()
        self.create_plot()
        self.create_sliders()
        self.create_buttons()
        self.create_tables()
        self.fill_layout()

    def create_window(self) -> None:
        """Creates a QT window."""
        self.window = QWidget()
        self.window.setWindowTitle("IK Solver - Widget")
        self.layout = QGridLayout(self.window)
        pg.setConfigOptions(antialias=True)

    def create_plot(self) -> None:
        """Creates a plot where we can visualize a pose."""
        self.plot_window = pg.GraphicsWindow()
        self.plot_window.setBackground("w")
        plot: pg.PlotItem = self.plot_window.addPlot()
        plot.setAspectLocked()
        plot.showGrid(x=True, y=True)

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

    def create_sliders(self) -> None:
        """Creates sliders to control all the poses."""
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

        self.slider_midpoint_shift = QSlider()
        self.slider_midpoint_shift.setOrientation(Qt.Horizontal)
        self.slider_midpoint_shift.setValue(50)
        self.slider_midpoint_shift.valueChanged.connect(self.update_midpoint_shift)

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
        self.horizontal_sliders_bottom.addWidget(self.slider_midpoint_shift, 0, 1)
        self.horizontal_sliders_bottom.addWidget(self.slider_midpoint_height, 0, 2)

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

    def create_buttons(self) -> None:
        """Create buttons to reset pose and toggle view of midpoints."""
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.reset)

        self.df_front_button = QCheckBox("Show midpoints")
        self.df_front_button.setChecked(True)
        self.df_front_button.clicked.connect(self.toggle_show_midpoint)

        self.buttons = QGridLayout()
        self.buttons.addWidget(self.reset_button, 0, 0)
        self.buttons.addWidget(self.df_front_button, 0, 1)

    def create_tables(self) -> None:
        """Create tables where we write the angles of all joints and the current settings."""
        self.table = QGridLayout()
        self.tables = {"last": pg.TableWidget(), "next": pg.TableWidget(), "settings": pg.TableWidget()}
        self.update_all_tables()
        for name in self.tables.keys():
            self.table.addWidget(self.tables[name], list(self.tables.keys()).index(name), 0)

    def fill_layout(self) -> None:
        """Fill the layout of the window with all the created elements."""
        self.layout.addLayout(self.vertical_sliders, 0, 0)
        self.layout.addWidget(self.plot_window, 0, 1)
        self.layout.addLayout(self.horizontal_sliders, 1, 1)
        self.layout.addLayout(self.table, 0, 2)
        self.layout.addLayout(self.buttons, 1, 2)

    @property
    def midpoint_height(self) -> float:
        """Returns the midpoint hegiht based on foot locations and midpoint height slider.

        Returns:
            float: the midpoint height.
        """
        return self.sliders["mid"]["height"] + max(-self.sliders["last"]["y"], self.sliders["next"]["y"], 0.0)

    def update_last_x(self, value) -> None:
        """Update the x value of last pose."""
        self.sliders["last"]["x"] = (1 - (value / 99)) * (X_MAX - X_MIN) + X_MIN
        self.update_pose("last")
        self.update_midpoints_and_tables()

    def update_next_x(self, value) -> None:
        """Update the x value of next pose."""
        self.sliders["next"]["x"] = (value / 99) * (X_MAX - X_MIN) + X_MIN
        self.update_pose("next")
        self.update_midpoints_and_tables()

    def update_last_y(self, value) -> None:
        """Update the y value of last pose."""
        self.sliders["last"]["y"] = (1 - (value / 99)) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose("last")
        self.update_midpoints_and_tables()

    def update_next_y(self, value) -> None:
        """Update the y value of next pose."""
        self.sliders["next"]["y"] = (value / 99) * (Y_MAX - Y_MIN) + Y_MIN
        self.update_pose("next")
        self.update_midpoints_and_tables()

    def update_midpoints(self) -> None:
        """Calls the methods to update the midpoints."""
        if self.show_midpoints:
            self.update_pose("mid_pre")
            self.update_pose("mid_post")

    def update_midpoints_and_tables(self) -> None:
        """Calls the methods to update the midpoints and the tables."""
        self.reset_midpoint_fraction()
        self.update_midpoints()
        self.update_all_tables()

    def update_midpoint_fraction(self, value) -> None:
        """Update the fraction of the step at which we want to show a pose."""
        self.sliders["mid"]["fraction"] = value / 100
        self.plot_pose("mid", self.pose_from_dynamic_joint_trajectory())
        self.update_all_tables()

    def update_midpoint_deviation(self, value) -> None:
        """Update the deviation used to create the midpoints."""
        self.sliders["mid"]["deviation"] = value / 100
        self.update_midpoints()
        self.reset_midpoint_fraction()
        self.update_all_tables()

    def update_midpoint_shift(self, value) -> None:
        """Update the shift of the point where the midpoints are created around."""
        self.sliders["mid"]["shift"] = (value - 50) / 100
        self.update_midpoints()
        self.reset_midpoint_fraction()
        self.update_all_tables()

    def update_midpoint_height(self, value) -> None:
        """Update the height used to create the midpoints."""
        self.sliders["mid"]["height"] = value / 100 * MIDPOINT_HEIGHT
        self.update_midpoints()
        self.reset_midpoint_fraction()
        self.update_all_tables()

    def reset(self) -> None:
        """Reset poses and corresponding sliders."""
        self.slider_last_x.setValue(99)
        self.slider_next_x.setValue(0)
        self.slider_last_y.setValue(50)
        self.slider_next_y.setValue(50)
        self.slider_midpoint_shift.setValue(50)
        for pose in ["last", "next"]:
            for axis in ["x", "y"]:
                self.sliders[pose][axis] = 0
        self.update_poses()
        self.reset_midpoint_poses()
        self.update_all_tables()

    def reset_midpoint_fraction(self) -> None:
        """Resets the slider and clears the plot."""
        self.slider_midpoint_fraction.setValue(0)
        self.plots["mid"].setData(x=[], y=[])

    def reset_midpoint_poses(self) -> None:
        """Clears the plots for the midpoint poses."""
        for pose_name in ["mid_pre", "mid_post"]:
            self.plots[pose_name].setData(x=[], y=[])

    def toggle_show_midpoint(self) -> None:
        """Toggles whether midpoints are shown."""
        self.show_midpoints = not self.show_midpoints
        if not self.show_midpoints:
            for plot in ["mid_pre", "mid_post", "mid_of_step", "mid_pre_ankle", "mid_post_ankle"]:
                self.plots[plot].setData(x=[], y=[])
        else:
            for pose in ["mid_pre", "mid_post"]:
                self.update_pose(pose)

    def get_midpoint_fraction_based_on_deviation(self, pose_name: str) -> float:
        """Get the midpoint fraction based on the deviation.

        Args:
            pose_name (str): the name of the pose.

        Returns:
            float: the fraction of the midpoint.
        """
        if pose_name == "mid_pre":
            return min(
                max(0.5 * (1 - self.sliders["mid"]["deviation"]) + self.sliders["mid"]["shift"], MIN_MIDPOINT_FRACTION),
                MAX_MIDPOINT_FRACTION,
            )
        elif pose_name == "mid_post":
            return max(
                min(0.5 * (1 + self.sliders["mid"]["deviation"]) + self.sliders["mid"]["shift"], MAX_MIDPOINT_FRACTION),
                MIN_MIDPOINT_FRACTION,
            )
        else:
            return 0.5 + self.sliders["mid"]["shift"]

    def setpoint_time(self, pose_name: str) -> float:
        """Returns the setpoint time for the given pose.

        Args:
            pose_name (str): the name of the pose.

        Returns:
            float: the setpoint time for the given pose.
        """
        if pose_name == "last":
            return 0.0
        elif pose_name == "next":
            return self.duration
        else:
            return self.duration * self.get_midpoint_fraction_based_on_deviation(pose_name)

    @property
    def duration(self):
        """Returns the duration, scaled to height by GaitPreprocessor method."""
        return GaitPreprocessor.get_duration_scaled_to_height(
            DEFAULT_DURATION, self.sliders["next"]["y"]
        )

    def interpolate_trough_points(self) -> None:
        """Interpolates setpoints to joint trajectory for every joint and the foot rotation."""
        self.joint_trajectories = []
        for n in range(NUMBER_OF_JOINTS + 1):
            setpoint_list = []

            pose_names = list(self.poses.keys())
            if self.get_midpoint_fraction_based_on_deviation(
                "mid_pre"
            ) >= self.get_midpoint_fraction_based_on_deviation("mid_post"):
                pose_names.remove("mid_post")

            for pose_name in pose_names:
                time = Duration(self.setpoint_time(pose_name))
                if n < NUMBER_OF_JOINTS:
                    position = (
                        self.poses[pose_name].pose_left[n]
                        if pose_name == "last"
                        else self.poses[pose_name].pose_right[n]
                    )
                else:
                    position = 0.0 if pose_name == "last" else self.poses[pose_name].rot_foot1
                setpoint_list.append(Setpoint(time, position, 0.0))
            self.joint_trajectories.append(DynamicJointTrajectory(setpoint_list))

    def pose_from_dynamic_joint_trajectory(self) -> Pose:
        """Creates a pose from all joints dynamic joint trajectories, on time given by the fraction slider.

        Returns:
            Pose: a pose object created form the joint_trajectories and the fraction slider.
        """
        pose_list = []
        for n in range(NUMBER_OF_JOINTS):
            pose_list.append(
                self.joint_trajectories[n]
                .get_interpolated_setpoint(self.sliders["mid"]["fraction"] * self.duration)
                .position
            )
        pose = Pose(IK_SOLVER_PARAMETERS, pose_list)
        pose.rot_foot1 = max(
            self.joint_trajectories[-1]
            .get_interpolated_setpoint(self.sliders["mid"]["fraction"] * self.duration)
            .position,
            0.0,  # Since negative foot rotation is not possible.
        )
        return pose

    def update_pose(self, pose_name: str) -> None:
        """Update the given pose.

        Args:
            pose_name (str): name of the pose.
        """
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

    def plot_pose(self, pose_name: str, pose: Pose) -> None:
        """Plots the given pose.

        Args:
            pose_name (str): name of the pose.
            pose (Pose): the pose to plot.
        """
        positions = list(pose.calculate_joint_positions().values())

        # select color based on limit check:
        try:
            check_on_limits(pose.pose_left)
            self.plots[pose_name].setPen(self.pose_colours[pose_name])
        except (PositionSoftLimitError) as value_error:
            print(value_error)
            self.plots[pose_name].setPen("r")

        # shift positions to have toes of stand leg at (0,0):
        if pose_name == "last":
            positions = [pos - positions[-1] for pos in positions]
        else:
            positions = [pos - positions[0] for pos in positions]

        positions_x = [pos[0] for pos in positions]
        positions_y = [pos[1] for pos in positions]
        self.plots[pose_name].setData(x=positions_x, y=positions_y)

        if self.show_midpoints:
            self.update_midpoint_ankle_postions()
        self.interpolate_trough_points()

    def update_poses(self) -> None:
        """Update the start and end poses."""
        for pose in ["last", "next"]:
            self.update_pose(pose)

    def update_midpoint_ankle_postions(self) -> None:
        """Update the two points at which the ankle is desired for the two midpoints."""
        ankle_x_positions = []
        for pose in ["mid_pre", "mid_post"]:
            fraction = self.get_midpoint_fraction_based_on_deviation(pose)
            x = self.poses["last"].get_ankle_mid_x(fraction, self.poses["next"])
            x -= self.poses["next"].pos_toes1[0]  # Shift since we center around toes.
            ankle_x_positions.append(x)
            y = self.midpoint_height
            self.plots[pose + "_ankle"].setData(x=[x], y=[y])

        mid_of_step = sum(ankle_x_positions) / len(ankle_x_positions)
        self.plots["mid_of_step"].setData(x=[mid_of_step], y=[0])

    def update_table_joint_angles(self) -> None:
        """Update the tables that show the joint angles."""
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

    def update_table_settings(self) -> None:
        """Update the table that shows the current settings."""
        data = [list(self.sliders["mid"].values())]
        self.tables["settings"].setData(data)
        self.tables["settings"].setHorizontalHeaderLabels(["frac", "dev", "shift", "height"])
        self.tables["settings"].verticalHeader().hide()

    def update_all_tables(self) -> None:
        """Updates all tables."""
        self.update_table_joint_angles()
        self.update_table_settings()

    def show(self) -> None:
        """Show the tool."""
        self.window.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    live_widget = LiveWidget()
    live_widget.show()
    sys.exit(app.exec_())

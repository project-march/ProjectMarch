"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import numpy as np
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout, \
    QLabel, QSlider, QDoubleSpinBox, QHBoxLayout, QPushButton, \
    QCheckBox
from enum import Enum

class Joint:

    def __init__(self, name: str, max_freq: float) -> None:
        self.name = name
        self.actual_position = 0.0   # rad
        self.desired_position = 0.0  # rad
        self.zero_position = 0.0  # rad
        self.amplitude = 0.0  # rad
        self.frequency = 1.0    # Hz
        self.max_frequency = max_freq # Hz
        self.dt = 1.0 / self.max_frequency # s
        self.linear_interpolation_velocity = 0.5 # rad/s
        
        self.is_selected = False
        self.is_flipped = False
        self.counter = 0
        self.new_joints_selected = False
        self.is_running = False

    def create_layout(self) -> QVBoxLayout:
        layout = QHBoxLayout()

        # Create label
        label_layout = QVBoxLayout()
        label = QLabel(self.name)
        label.setStyleSheet("font-weight: bold;")
        label.setAlignment(Qt.AlignCenter)
        label_layout.addWidget(label)
        self.select_box = QCheckBox()
        self.select_box.setChecked(False)
        self.select_box.setStyleSheet("margin-left: 50%; margin-right: 50%;")
        self.select_box.stateChanged.connect(self.update_is_selected)
        label_layout.addWidget(self.select_box)

        # Create actual joint position field
        actual_position_layout = QVBoxLayout()
        actual_position_layout.addWidget(QLabel("Actual position (rad)"))
        self.actual_position_widget = QDoubleSpinBox()
        self.actual_position_widget.setDecimals(9)
        self.actual_position_widget.setReadOnly(True)
        self.actual_position_widget.setRange(-np.pi, np.pi)
        self.actual_position_widget.setStyleSheet(self.remove_arrows())
        self.actual_position_widget.valueChanged.connect(self.update_actual_pos)
        actual_position_layout.addWidget(self.actual_position_widget)

        # Create desired joint position field
        desired_position_layout = QVBoxLayout()
        desired_position_layout.addWidget(QLabel("Desired pos."))
        self.desired_position_widget = QDoubleSpinBox()
        self.desired_position_widget.setDecimals(3)
        self.actual_position_widget.setReadOnly(True)
        self.desired_position_widget.setSingleStep(5e-2)
        self.desired_position_widget.setRange(-np.pi, np.pi)
        self.desired_position_widget.valueChanged.connect(self.update_desired_pos)
        desired_position_layout.addWidget(self.desired_position_widget)

        # Create zero position field
        zero_position_layout = QVBoxLayout()
        zero_position_layout.addWidget(QLabel("Target position (rad)"))
        self.zero_position_widget = QDoubleSpinBox()
        self.zero_position_widget.setDecimals(3)
        self.zero_position_widget.setSingleStep(5e-3)
        self.zero_position_widget.setRange(-np.pi, np.pi)
        self.zero_position_widget.setDisabled(True)
        self.zero_position_widget.valueChanged.connect(self.update_zero_pos)
        zero_position_layout.addWidget(self.zero_position_widget)

        # Create amplitude field
        amplitude_layout = QVBoxLayout()
        amplitude_layout.addWidget(QLabel("Amplitude (rad)"))
        self.amplitude_widget = QDoubleSpinBox()
        self.amplitude_widget.setDecimals(3)
        self.amplitude_widget.setSingleStep(5e-2)
        self.amplitude_widget.setRange(0, np.pi)
        self.amplitude_widget.setDisabled(True)
        self.amplitude_widget.valueChanged.connect(self.update_amplitude)
        amplitude_layout.addWidget(self.amplitude_widget)
        
        # # Create min amplitude field
        # min_amplitude_layout = QVBoxLayout()
        # min_amplitude_layout.addWidget(QLabel("Min amplitude (rad)"))
        # self.min_amplitude_widget = QDoubleSpinBox()
        # self.min_amplitude_widget.setDecimals(3)
        # self.min_amplitude_widget.setSingleStep(5e-2)
        # self.min_amplitude_widget.setRange(-np.pi, 0)
        # self.min_amplitude_widget.setDisabled(True)
        # self.min_amplitude_widget.setAutoFillBackground(False)
        # self.min_amplitude_widget.valueChanged.connect(self.update_min_amplitude)
        # min_amplitude_layout.addWidget(self.min_amplitude_widget)

        # Create frequency field
        frequency_layout = QVBoxLayout()
        frequency_layout.addWidget(QLabel("Period (s)"))
        self.frequency_widget = QDoubleSpinBox()
        self.frequency_widget.setDecimals(3)
        self.frequency_widget.setSingleStep(5e-2)
        self.frequency_widget.setDisabled(True)
        self.frequency_widget.setMinimum(1.0)
        self.frequency_widget.setValue(1.0)
        # self.frequency_widget.setStyleSheet(self.remove_arrows())
        self.frequency_widget.valueChanged.connect(self.update_frequency)
        frequency_layout.addWidget(self.frequency_widget)

        layout.addLayout(label_layout)
        layout.addLayout(actual_position_layout)
        # layout.addLayout(desired_position_layout)
        layout.addLayout(zero_position_layout)
        layout.addLayout(amplitude_layout)
        # layout.addLayout(min_amplitude_layout)
        layout.addLayout(frequency_layout)
        return layout
    
    def update_is_selected(self, state: int) -> None:
        self.is_selected = state == Qt.Checked
        if self.is_selected:
            self.zero_position_widget.setDisabled(False)
            self.amplitude_widget.setDisabled(False)
            # self.min_amplitude_widget.setDisabled(False)
            self.frequency_widget.setDisabled(False)
            self.disable_run_button_callback()
        else:
            self.zero_position_widget.setDisabled(True)
            self.amplitude_widget.setDisabled(True)
            # self.min_amplitude_widget.setDisabled(True)
            self.frequency_widget.setDisabled(True)

    def update_actual_pos(self, pos: float) -> None:
        self.actual_position_widget.setValue(pos)
        self.actual_position = pos

    def update_desired_pos(self, pos: float) -> None:
        self.desired_position_widget.setValue(pos)

    def update_zero_pos(self, pos: float) -> None:
        self.zero_position_widget.setValue(pos)
        self.zero_position = pos
        self.disable_run_button_callback()

    def update_amplitude(self, amplitude: float) -> None:
        self.amplitude_widget.setValue(amplitude)
        self.amplitude = amplitude

    def update_min_amplitude(self, amplitude: float) -> None:
        self.min_amplitude_widget.setValue(amplitude)
        self.min_amplitude = amplitude

    def update_frequency(self, period: float) -> None:
        self.frequency_widget.setValue(period)
        if np.isclose(period, 0.0):
            self.frequency = 0.0
            return
        self.frequency = 1.0 / period

    def remove_arrows(self) -> str:
        return """ QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
            width: 0px;
            height: 0px;
            border: none;
        } """
    
    def disable_widgets(self) -> None:
        self.select_box.setDisabled(True)
        self.desired_position_widget.setDisabled(True)
        self.zero_position_widget.setDisabled(True)
        self.amplitude_widget.setDisabled(True)
        self.frequency_widget.setDisabled(True)

    def enable_widgets(self) -> None:
        self.select_box.setDisabled(False)
        self.desired_position_widget.setDisabled(False)
        self.zero_position_widget.setDisabled(False)
        self.amplitude_widget.setDisabled(False)
        self.frequency_widget.setDisabled(False)

    def set_disable_run_button_callback(self, disable_run_button_callback) -> None:
        self.disable_run_button_callback = disable_run_button_callback

    def update_desired_pos(self) -> float:
        if not self.is_selected:
            return self.actual_position
        t = self.counter * self.dt
        desired_position = self.zero_position + self.amplitude * np.sin(2 * np.pi * self.frequency * t)
        self.counter += 1
        if t + self.dt > 1 / self.frequency:
            self.counter = 1
        return desired_position


class WindowState(Enum):
    IDLE = 0
    RUNNING = 1
    STOPPED = 2
    TARGETS_SET = 3
    ERROR = 4

class Window(QMainWindow):

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('Test Joints GUI')
        self.setGeometry(100, 100, 1280, 768)
        self.is_running = False
        self.joints = None
        self.state = "Idle"
        self.linear_desired_positions = []

    def create_layout(self, joint_names: list, max_freq: float) -> None:
        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)

        self.joint_layout = QVBoxLayout()
        self.centralWidget.setLayout(self.joint_layout)

        # Create layout for joints
        self.joints = [Joint(name, max_freq) for name in joint_names]
        for joint in self.joints:
            joint.set_disable_run_button_callback(self.disable_run_button)
            self.joint_layout.addLayout(joint.create_layout())
        self.joint_layout.addStretch()

        # Create layout for menu
        menu_layout = QHBoxLayout()
        self.menu_label = QLabel("State: Idle")
        self.menu_label.setAlignment(Qt.AlignCenter)
        self.menu_label.setStyleSheet("font-size: 32px; font-weight: bold;")
        menu_layout.addStretch()
        menu_layout.addWidget(self.menu_label)
        menu_layout.addStretch()
        menu_layout.addStretch()

        self.run_button = QPushButton("Start Oscillation")
        self.run_button.setDisabled(True)
        self.run_button.clicked.connect(self.run_button_callback)

        self.stop_button = QPushButton("Stop")
        self.stop_button.setDisabled(True)
        self.stop_button.clicked.connect(self.stop_button_callback)

        self.reset_button = QPushButton("Set Targets")
        self.reset_button.setDisabled(False)
        self.reset_button.clicked.connect(self.reset_button_callback)

        menu_layout.addWidget(self.run_button)
        menu_layout.addStretch()
        menu_layout.addWidget(self.stop_button)
        menu_layout.addStretch()
        menu_layout.addWidget(self.reset_button)
        menu_layout.addStretch()
        self.joint_layout.addLayout(menu_layout)

    def run_button_callback(self) -> None:
        self.update_state(WindowState.RUNNING)
        self.run_button.setDisabled(True)
        self.stop_button.setDisabled(False)
        self.reset_button.setDisabled(True)
        for joint in self.joints:
            joint.disable_widgets()
        self.is_running = True

    def stop_button_callback(self) -> None:
        self.update_state(WindowState.STOPPED)
        self.stop_button.setDisabled(True)
        self.run_button.setDisabled(True)
        self.reset_button.setDisabled(False)
        for joint in self.joints:
            joint.select_box.setDisabled(False)
            if joint.is_selected:
                joint.enable_widgets()
        self.is_running = False

    def reset_button_callback(self) -> None:
        self.update_state(WindowState.TARGETS_SET)
        self.run_button.setDisabled(False)
        self.stop_button.setDisabled(True)
        desired_joint_positions = []
        for joint in self.joints:
            joint.select_box.setDisabled(False)
            joint.counter = 0
            joint.is_flipped = False
            if joint.is_selected:
                desired_joint_positions.append(joint.zero_position)
                joint.enable_widgets()
            else:
                desired_joint_positions.append(joint.actual_position)
        self.linearize_desired_positions(desired_joint_positions)
        self.is_running = True

    def update_actual_positions(self, names: list, positions: list) -> None:
        for joint in self.joints:
            if joint.name in names:
                joint.update_actual_pos(positions[names.index(joint.name)])

    def get_desired_positions(self) -> list:
        return [joint.update_desired_pos() for joint in self.joints]
    
    def set_timer_callbacks(self, enable_callback, disable_callback) -> None:
        self.enable_timer = enable_callback
        self.disable_timer = disable_callback

    def set_publish_callback(self, publish_callback) -> None:
        self.publish_callback = publish_callback

    def disable_run_button(self) -> None:
        self.run_button.setDisabled(True)

    def linearize_desired_positions(self, desired_positions: list) -> list:
        current_positions = [joint.actual_position for joint in self.joints]
        self.linear_desired_positions = np.linspace(current_positions, desired_positions, 100).tolist()

    def update_state(self, state: WindowState) -> None:
        self.state = state
        if state == WindowState.IDLE:
            self.menu_label.setText("State: Idle")
        elif state == WindowState.RUNNING:
            self.menu_label.setText("State: Running")
        elif state == WindowState.STOPPED:
            self.menu_label.setText("State: Stopped")
        elif state == WindowState.TARGETS_SET:
            self.menu_label.setText("State: Targets Set")
        elif state == WindowState.ERROR:
            self.menu_label.setText("State: Error")
            
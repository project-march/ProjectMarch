import math
import os
import subprocess

from march_shared_classes.utilities.side import Side

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QKeySequence
from python_qt_binding.QtWidgets import (
    QApplication,
    QDialog,
    QFileDialog,
    QFrame,
    QHeaderView,
    QMessageBox,
    QShortcut,
    QWidget,
    QInputDialog,
)
import rospkg
import rospy
from rviz import bindings as rviz
from sensor_msgs.msg import JointState
from tf import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from .joint_plot import JointPlot
from .joint_table_controller import JointTableController
from .side_subgait_view import SideSubgaitView
from .time_slider_thread import TimeSliderThread


class GaitGeneratorView(QWidget):
    def __init__(self):
        super(GaitGeneratorView, self).__init__()

        self.joint_state_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )

        current_file_path = __file__.split("/")
        path = "/"
        for directory in current_file_path:
            path = os.path.join(path, directory)
            if directory == "ros2":
                break
        self.ros2_path = path

        self.joint_widgets = {}
        self.tf_listener = TransformListener()

        ui_file = os.path.join(
            rospkg.RosPack().get_path("march_rqt_gait_generator"),
            "resource",
            "gait_generator.ui",
        )
        loadUi(ui_file, self)

        self.rviz_frame = self.create_rviz_frame()
        self.RvizFrame.layout().addWidget(self.rviz_frame, 1, 0, 1, 3)

        self.gait_type_combo_box.addItems(["walk_like", "sit_like", "stairs_like"])

        previous_subgait_view = SideSubgaitView(widget=self.previous_subgait_container)
        next_subgait_view = SideSubgaitView(widget=self.next_subgait_container)
        self.side_subgait_view = {
            "previous": previous_subgait_view,
            "next": next_subgait_view,
        }

        self.initialize_shortcuts()

    def initialize_shortcuts(self):
        self.ctrl_z = QShortcut(QKeySequence(self.tr("Ctrl+Z")), self)
        self.ctrl_shift_z = QShortcut(QKeySequence(self.tr("Ctrl+Shift+Z")), self)

    # Called by build_ui
    def create_rviz_frame(self):
        frame = rviz.VisualizationFrame()
        frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(
            config,
            os.path.join(
                rospkg.RosPack().get_path("march_rqt_gait_generator"),
                "resource",
                "cfg.rviz",
            ),
        )
        frame.load(config)

        # Hide irrelevant Rviz details
        frame.setMenuBar(None)
        frame.setStatusBar(None)
        frame.setHideButtonVisibility(False)
        return frame

    # Called by __init__ and import_gait.
    def load_gait_into_ui(self, gait):
        self.time_slider.setRange(0, 100 * gait.duration)

        self.gait_type_combo_box.setCurrentText(gait.gait_type)
        self.gait_name_line_edit.setText(gait.gait_name)
        self.subgait_name_line_edit.setText(gait.subgait_name)
        self.version_name_line_edit.setText(gait.version)
        self.description_line_edit.setText(gait.description)

        # Block signals on the duration edit to prevent a reload of the joint settings
        self.duration_spin_box.blockSignals(True)
        self.duration_spin_box.setValue(gait.duration)
        self.duration_spin_box.blockSignals(False)

        self.load_joint_plots(gait.joints)
        self.publish_preview(gait, time=0)

    # Methods below are called by load_gait_into_ui.
    def update_time_sliders(self, time):
        graphics_layouts = self.JointSettingContainer.findChildren(
            pg.GraphicsLayoutWidget
        )
        for graphics_layout in graphics_layouts:
            joint_settings_plot = graphics_layout.getItem(0, 0)
            joint_settings_plot.update_time_slider(time)

    def load_joint_plots(self, joints):
        layout = self.JointSettingContainer.layout()
        for i in reversed(range(layout.count())):
            widget = layout.itemAt(i).widget()
            layout.removeWidget(widget)
            widget.setParent(None)

        for joint in joints:
            self.joint_widgets[joint.name] = self.create_joint_plot_widget(joint)
            row = rospy.get_param("/joint_layout/" + joint.name + "/row", -1)
            column = rospy.get_param("/joint_layout/" + joint.name + "/column", -1)
            if row == -1 or column == -1:
                rospy.logerr(
                    "Could not load the layout for joint %s. Please check config/layout.yaml",
                    joint.name,
                )
                continue
            self.JointSettingContainer.layout().addWidget(
                self.joint_widgets[joint.name], row, column
            )

    def create_joint_plot_widget(self, joint):
        joint_setting_file = os.path.join(
            rospkg.RosPack().get_path("march_rqt_gait_generator"),
            "resource",
            "joint_setting.ui",
        )

        joint_plot_widget = QFrame()
        loadUi(joint_setting_file, joint_plot_widget)

        show_velocity_plot = self.velocity_plot_check_box.isChecked()
        show_effort_plot = self.effort_plot_check_box.isChecked()
        joint_plot = JointPlot(joint, show_velocity_plot, show_effort_plot)
        joint_plot_widget.Plot.addItem(joint_plot)

        # Disable scrolling horizontally
        joint_plot_widget.Table.horizontalScrollBar().setDisabled(True)
        joint_plot_widget.Table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch
        )
        joint_plot_widget.Table.controller = JointTableController(
            joint_plot_widget.Table, joint
        )

        return joint_plot_widget

    # Methods below are called during runtime
    def publish_preview(self, subgait, time):
        joint_state = JointState()
        joint_state.header.stamp = rospy.get_rostime()

        for joint in subgait.joints:
            joint_state.name.append(joint.name)
            joint_state.position.append(joint.get_interpolated_setpoint(time).position)
        self.joint_state_pub.publish(joint_state)
        self.set_feet_distances()

    def set_feet_distances(self):
        try:
            # The translation from the right foot to the left foot is the position of the
            # left foot from the right foot's frame of reference.
            (trans_left, rot_right) = self.tf_listener.lookupTransform(
                "/foot_right", "/foot_left", rospy.Time(0)
            )
            (trans_right, rot_left) = self.tf_listener.lookupTransform(
                "/foot_left", "/foot_right", rospy.Time(0)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        self.height_left_line_edit.setText(f"{trans_left[2]:.3f}")
        self.height_right_line_edit.setText(f"{trans_right[2]:.3f}")
        self.heel_distance_line_edit.setText(
            f"{math.sqrt(trans_left[0] ** 2 + trans_left[2] ** 2)}:.3f"
        )

    def message(self, title=None, msg=None):
        QMessageBox.question(self, title, msg, QMessageBox.Ok)

    def yes_no_question(self, title=None, msg=None):
        answer = QMessageBox.question(
            self, title, msg, QMessageBox.Yes | QMessageBox.No
        )
        return answer == QMessageBox.Yes

    def open_file_dialogue(self):
        return QFileDialog.getOpenFileName(
            self,
            "Select a subgait to import.",
            os.path.join(self.ros2_path, "src", "gaits", "march_gait_files"),
            "March Subgait (*.subgait)",
        )

    def open_directory_dialogue(self):
        return QFileDialog.getExistingDirectory(
            None,
            "Select a directory to save gaits. Directory must be "
            "a subdirectory of march_gait_files or be named resources.",
            os.path.join(self.ros2_path, "src", "gaits", "march_gait_files"),
        )

    def get_inverse_kinematics_setpoints_input(self):
        """Asks the user the needed inputs, returns these plus a flag indicating cancellation of any input."""
        input_dictionary = {}

        (
            input_dictionary,
            cancelled,
        ) = self.get_inverse_kinematics_setpoints_input_settings(input_dictionary)

        if cancelled:
            return input_dictionary, cancelled

        (
            input_dictionary,
            cancelled,
        ) = self.get_inverse_kinematics_setpoints_input_foot_state(input_dictionary)

        return input_dictionary, cancelled

    def get_inverse_kinematics_setpoints_input_settings(self, input_dictionary):
        """Asks the user to specify certain settings needed for the inverse kinematics setpoints."""
        input_name = "foot_side"
        output_item, ok = QInputDialog.getItem(
            self, "enter foot side choice", input_name, ["right", "left"], 0, False
        )
        if ok:
            if output_item == "right":
                input_dictionary[input_name] = Side.right
            else:
                input_dictionary[input_name] = Side.left
        else:
            return input_dictionary, True

        input_name = "z_axis"
        output_item, ok = QInputDialog.getItem(
            self,
            "enter z axis choice",
            input_name,
            ["from ground upwards", "from hip downwards"],
            0,
            False,
        )
        if ok:
            input_dictionary[input_name] = output_item
        else:
            return input_dictionary, True

        (input_dictionary, cancelled,) = self.get_yes_no_input(
            "Do you want to specify a desired foot velocity?",
            "set_velocity",
            input_dictionary,
        )

        if cancelled:
            return input_dictionary, cancelled

        (input_dictionary, cancelled,) = self.get_yes_no_input(
            "Do you want to specify the y coordinate of the foot?",
            "set_y_coordinate",
            input_dictionary,
        )

        return input_dictionary, cancelled

    def get_yes_no_input(self, input_text, asked_variable_name, input_dictionary):
        """General input grabber for yes no questions."""
        output_item, ok = QInputDialog.getItem(
            self, input_text, asked_variable_name, ["no", "yes"], 0, False
        )
        if ok:
            if output_item == "no":
                input_dictionary[asked_variable_name] = False
            else:
                input_dictionary[asked_variable_name] = True
            return input_dictionary, False
        else:
            return input_dictionary, True

    def get_inverse_kinematics_setpoints_input_foot_state(self, input_dictionary):
        """Asks the inputs for the foot coordinate needed for the inverse kinematics setpoints."""
        input_dialogue = QInputDialog(self)
        input_dialogue.setDoubleDecimals(2)
        input_dialogue.setDoubleRange(-100.0, 100.0)
        input_dialogue.resize(300, 100)

        input_dialogue.setInputMode(QInputDialog.DoubleInput)
        double_variable_names = [
            "x_coordinate_cm",
            "y_coordinate_cm",
            "z_coordinate_cm",
            "time_s",
        ]

        if input_dictionary["set_velocity"]:
            double_variable_names.extend(
                ["x_velocity_cm_per_s", "y_velocity_cm_per_s", "z_velocity_cm_per_s"]
            )

        for double_variable_name in double_variable_names:
            if (
                not input_dictionary["set_y_coordinate"]
                and double_variable_name == "y_coordinate_cm"
            ):
                input_dictionary["y_coordinate_cm"] = 0
                continue

            # Form a string explaining the variable to the user from the variable name
            if double_variable_name == "time_s":
                input_text = f"Enter desired {double_variable_name} of foot position"
            elif double_variable_name.find("coordinate") != -1:
                input_text = (
                    f"Enter desired deviation from default foot "
                    f"position in {double_variable_name[0]} direction"
                )
            else:
                input_text = f"Enter desired velocity of foot in {double_variable_name[0]} direction"

            (input_dictionary, cancelled,) = self.get_double_input(
                input_dialogue, input_text, double_variable_name, input_dictionary
            )
            if cancelled:
                return input_dictionary, cancelled

        return input_dictionary, False

    def get_double_input(
        self, input_dialogue, input_text, asked_variable_name, input_dictionary
    ):
        """General input grabber for doubles for the filling of the inverse kinematics setpoints input."""
        input_dialogue.setLabelText(input_text)
        ok = input_dialogue.exec_()
        resulting_number = input_dialogue.doubleValue()
        if ok:
            input_dictionary[asked_variable_name] = resulting_number
            return input_dictionary, False
        else:
            return input_dictionary, True

    @QtCore.pyqtSlot(int)
    def update_main_time_slider(self, time):
        self.time_slider.setValue(time)

    def update_joint_widgets(self, joints):
        for joint in joints:
            self.update_joint_widget(joint)

    def update_joint_widget(self, joint, update_table=True):
        plot = self.joint_widgets[joint.name].Plot.getItem(0, 0)
        plot.plot_item.blockSignals(True)
        plot.update_setpoints(
            joint,
            show_velocity_plot=self.velocity_plot_check_box.isChecked(),
            show_effort_plot=self.effort_plot_check_box.isChecked(),
        )
        plot.plot_item.blockSignals(False)

        if update_table:
            table = self.joint_widgets[joint.name].Table
            table.blockSignals(True)
            table.controller.update_setpoints(joint)
            table.blockSignals(False)

    def set_duration_spinbox(self, duration):
        self.duration_spin_box.blockSignals(True)
        self.duration_spin_box.setValue(duration)
        self.duration_spin_box.blockSignals(False)

    @staticmethod
    def notify(title, message):
        subprocess.Popen(["notify-send", str(title), str(message)])

    @property
    def control_button(self):
        return QtCore.Qt.ControlModifier

    @staticmethod
    def create_time_slider_thread(current, playback_speed, max_time):
        return TimeSliderThread(current, playback_speed, max_time)

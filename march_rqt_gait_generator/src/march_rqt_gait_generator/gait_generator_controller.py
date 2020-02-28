import math
import os

from numpy_ringbuffer import RingBuffer
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox,
                                         QFileDialog, QFrame, QHeaderView,
                                         QLineEdit, QMessageBox, QPushButton,
                                         QSlider, QSpinBox, QWidget)
from qt_gui.plugin import Plugin
import rospkg
import rospy
import rviz
from sensor_msgs.msg import JointState
from tf import (ConnectivityException, ExtrapolationException, LookupException,
                TransformListener)
from trajectory_msgs.msg import JointTrajectory
from urdf_parser_py import urdf

from . import user_interface_controller
from .joint_setting_plot import JointSettingPlot
from .model.modifiable_setpoint import ModifiableSetpoint
from .model.modifiable_subgait import ModifiableSubgait
from .time_slider_thread import TimeSliderThread


class GaitGeneratorController(object):

    def __init__(self, view):
        self.view = view

        # Default values
        self.gait_publisher = None
        self.topic_name = ''
        self.gait_directory = None
        self.playback_speed = 100
        self.time_slider_thread = None
        self.set_topic_name(self.view.topic_name_line_edit.text())
        self.joint_changed_history = RingBuffer(capacity=100, dtype=list)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

        self.robot = urdf.Robot.from_parameter_server()
        self.gait = ModifiableSubgait.empty_subgait(self, self.robot)

        self.connect_buttons()


    # Called by __init__
    def connect_buttons(self):
        self.view.change_gait_directory_button.clicked.connect(self.change_gait_directory)
        self.view.import_gait_button.clicked.connect(self.import_gait)
        self.view.export_gait_button.clicked.connect(self.export_gait)

        self.view.publish_gait_button.clicked.connect(self.publish_gait)

        self.view.start_button.clicked.connect(self.start_time_slider_thread)
        self.view.stop_button.clicked.connect(self.stop_time_slider_thread)
        self.view.invert_button.clicked.connect(self.invert_gait)
        self.view.undo_button.clicked.connect(self.undo)
        self.view.redo_button.clicked.connect(self.redo)

        # Line edits / combo boxes / spin boxes
        self.view.gait_type_combo_box.currentTextChanged.connect(
            lambda text: self.gait.set_gait_type(text),
        )
        self.view.gait_name_line_edit.textChanged.connect(
            lambda text: self.gait.set_gait_name(text),
        )
        self.view.version_name_line_edit.textChanged.connect(
            lambda text: self.gait.set_version(text),
        )
        self.view.subgait_name_line_edit.textChanged.connect(
            lambda text: self.gait.set_subgait_name(text),
        )
        self.view.description_line_edit.textChanged.connect(
            lambda text: self.gait.set_description(text),
        )
        self.view.topic_name_line_edit.textChanged.connect(self.set_topic_name)
        self.view.playback_speed_spin_box.valueChanged.connect(self.set_playback_speed)
        self.view.duration_spin_box.setKeyboardTracking(False)
        self.view.duration_spin_box.valueChanged.connect(self.update_gait_duration)

        # Disable key inputs when mirroring is off.
        self.view.mirror_check_box.stateChanged.connect(
            lambda state: [
                self.view.mirror_key1_line_edit.setEnabled(state),
                self.view.mirror_key2_line_edit.setEnabled(state),
            ],
        )

    # Called by load_gait_into_ui and update_gait_duration.
    def create_joint_settings(self):
        joint_plots = []
        for urdf_joint in self.robot.joints:
            if urdf_joint.type != 'fixed':
                joint_name = urdf_joint.name

                joint = self.gait.get_joint(joint_name)
                if not joint:
                    continue
                joint_plots.append([joint_name, self.create_joint_setting(joint)])
        return joint_plots

    # Called 8 times by create_joint_settings.
    def create_joint_setting(self, joint):
        joint_setting_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource',
                                          'joint_setting.ui')

        joint_setting = QFrame()
        loadUi(joint_setting_file, joint_setting)

        show_velocity_plot = self.view.velocity_plot_check_box.isChecked()
        show_effort_plot = self.view.effort_plot_check_box.isChecked()
        joint_setting_plot = JointSettingPlot(joint, self.gait.duration, show_velocity_plot, show_effort_plot)
        joint_setting.Plot.addItem(joint_setting_plot)

        joint_setting.Table = user_interface_controller.update_table(
            joint_setting.Table, joint, self.gait.duration)
        # Disable scrolling horizontally
        joint_setting.Table.horizontalScrollBar().setDisabled(True)
        joint_setting.Table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        def update_joint_ui():
            user_interface_controller.update_ui_elements(
                joint, table=joint_setting.Table, plot=joint_setting_plot, duration=self.gait.duration,
                show_velocity_plot=self.view.velocity_plot_check_box.isChecked(),
                show_effort_plot=self.view.effort_plot_check_box.isChecked())
            self.view.publish_preview()

        def add_setpoint(joint, time, position, button):
            if button == QtCore.Qt.ControlModifier:
                joint.add_interpolated_setpoint(time)
            else:
                joint.add_setpoint(ModifiableSetpoint(time, position, 0))

        self.view.undo_button.clicked.connect(update_joint_ui)
        self.view.redo_button.clicked.connect(update_joint_ui)
        self.view.invert_button.clicked.connect(update_joint_ui)

        self.view.velocity_plot_check_box.stateChanged.connect(
            lambda: [
                update_joint_ui(),
            ])

        self.view.effort_plot_check_box.stateChanged.connect(
            lambda: [
                update_joint_ui(),
            ])

        # Connect a function to update the model and to update the table.
        joint_setting_plot.plot_item.sigPlotChanged.connect(
            lambda: [
                joint.set_setpoints(user_interface_controller.plot_to_setpoints(joint_setting_plot)),
                update_joint_ui(),
            ])

        joint_setting_plot.add_setpoint.connect(
            lambda time, position, button: [
                add_setpoint(joint, time, position, button),
                update_joint_ui(),
            ])

        joint_setting_plot.remove_setpoint.connect(
            lambda index: [
                joint.save_setpoints(),
                joint.remove_setpoint(index),
                update_joint_ui(),
            ])

        joint_setting.Table.itemChanged.connect(
            lambda: [
                joint.set_setpoints(user_interface_controller.table_to_setpoints(joint_setting.Table)),
                user_interface_controller.update_ui_elements(
                    joint, table=None, plot=joint_setting_plot, duration=self.gait.duration,
                    show_velocity_plot=self.view.velocity_plot_check_box.isChecked(),
                    show_effort_plot=self.view.effort_plot_check_box.isChecked()),
                self.view.publish_preview(),
            ])

        return joint_setting

    # Functions below are connected to buttons, text boxes, joint graphs etc.
    def publish_gait(self):
        trajectory = self.gait._to_joint_trajectory_msg()
        rospy.loginfo('Publishing trajectory to topic ' + self.topic_name)
        self.gait_publisher.publish(trajectory)

    def set_topic_name(self, topic_name):
        self.topic_name = topic_name
        self.gait_publisher = rospy.Publisher(topic_name,
                                              JointTrajectory, queue_size=10)

    def set_playback_speed(self, playback_speed):
        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.playback_speed = playback_speed
        rospy.loginfo('Changing playbackspeed to ' + str(self.playback_speed))

        if was_playing:
            self.start_time_slider_thread()

    def start_time_slider_thread(self):
        if self.time_slider_thread is not None:
            rospy.logdebug('Cannot start another time slider thread as one is already active')
            return

        current = self.view.time_slider.value()
        playback_speed = self.playback_speed
        max_time = self.view.time_slider.maximum()
        self.time_slider_thread = TimeSliderThread(current, playback_speed, max_time)
        self.time_slider_thread.update_signal.connect(self.view.update_main_time_slider)
        self.time_slider_thread.start()

    def stop_time_slider_thread(self):
        if self.time_slider_thread is not None:
            self.time_slider_thread.stop()
            self.time_slider_thread = None

    def update_gait_duration(self, duration):
        rescale_setpoints = self.view.scale_setpoints_check_box.isChecked()

        if self.gait.has_setpoints_after_duration(duration) and not rescale_setpoints:
            if not self.gait.has_multiple_setpoints_before_duration(duration):
                QMessageBox.question(self.view._widget, 'Could not update gait duration',
                                     'Not all joints have multiple setpoints before duration ' + str(duration),
                                     QMessageBox.Ok)
                return
            discard_setpoints = QMessageBox.question(self.view._widget, 'Gait duration lower than highest time setpoint',
                                                     'Do you want to discard any setpoints higher than the given '
                                                     'duration?',
                                                     QMessageBox.Yes | QMessageBox.No)
            if discard_setpoints == QMessageBox.No:
                self.view.duration_spin_box.setValue(self.gait.duration)
                return
        self.gait.set_duration(duration, rescale_setpoints)
        self.view.time_slider.setRange(0, 100 * self.gait.duration)

        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.view.build_joint_plots()

        if was_playing:
            self.start_time_slider_thread()

    def import_gait(self):
        file_name, f = QFileDialog.getOpenFileName(self.view._widget,
                                                   'Open Image',
                                                   os.getenv('HOME') + '/march_ws/src/gait-files/march_gait_files',
                                                   'March Subgait (*.subgait)')

        gait = ModifiableSubgait.from_file(self.robot, file_name, self)
        if gait is None:
            rospy.logwarn('Could not load gait %s', file_name)
            return
        self.gait = gait
        self.view.load_gait_into_ui(self.gait)

        self.gait_directory = '/'.join(file_name.split('/')[:-3])
        rospy.loginfo('Setting gait directory to %s', str(self.gait_directory))
        self.view.change_gait_directory_button.setText(self.gait_directory)

        # Clear undo and redo history
        self.joint_changed_history = RingBuffer(capacity=100, dtype=list)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

    def export_gait(self):
        should_mirror = self.view.mirror_check_box.isChecked()

        key_1 = self.view.mirror_key1_line_edit.text()
        key_2 = self.view.mirror_key2_line_edit.text()

        if should_mirror:
            mirror = self.gait.get_mirror(key_1, key_2)
            if mirror:
                self.export_to_file(mirror, self.get_gait_directory())
            else:
                user_interface_controller.notify('Could not mirror gait', 'Check the logs for more information.')
                return

        self.export_to_file(self.gait, self.get_gait_directory())

    @staticmethod
    def export_to_file(gait, gait_directory):
        if gait_directory is None or gait_directory == '':
            return

        # Name and version will be empty as it's stored in the filename.
        subgait_msg = gait.to_subgait_msg()

        output_file_directory = os.path.join(gait_directory,
                                             gait.gait_name.replace(' ', '_'),
                                             gait.subgait_name.replace(' ', '_'))
        output_file_path = os.path.join(output_file_directory,
                                        gait.version.replace(' ', '_') + '.subgait')

        file_exists = os.path.isfile(output_file_path)
        if file_exists:
            overwrite_file = QMessageBox.question(None, 'File already exists',
                                                  'Do you want to overwrite ' + str(output_file_path) + '?',
                                                  QMessageBox.Yes | QMessageBox.No)
            if overwrite_file == QMessageBox.No:
                return

        rospy.loginfo('Writing gait to ' + output_file_path)

        if not os.path.isdir(output_file_directory):
            os.makedirs(output_file_directory)

        with open(output_file_path, 'w') as output_file:
            output_file.write(str(subgait_msg))

        user_interface_controller.notify('Gait Saved', output_file_path)

    # Called by export_gait
    def get_gait_directory(self):
        if self.gait_directory is None:
            self.change_gait_directory()
        rospy.loginfo('Selected output directory ' + str(self.gait_directory))
        return self.gait_directory

    def change_gait_directory(self):
        self.gait_directory = str(QFileDialog.getExistingDirectory(None, 'Select a directory to save gaits'))
        if self.gait_directory == '':   # If directory dialogue is canceled
            self.gait_directory = None
            self.view.change_gait_directory_button.setText('Select a gait directory...')
        else:
            self.view.change_gait_directory_button.setText(self.gait_directory)

    def invert_gait(self):
        for joint in self.gait.joints:
            joint.invert()
        self.save_changed_joints(self.gait.joints)
        self.view.publish_preview()

    def undo(self):
        if not self.joint_changed_history:
            return

        joints = self.joint_changed_history.pop()
        for joint in joints:
            joint.undo()
        self.joint_changed_redo_list.append(joints)
        self.view.publish_preview()

    def redo(self):
        if not self.joint_changed_redo_list:
            return

        joints = self.joint_changed_redo_list.pop()
        for joint in joints:
            joint.redo()
        self.joint_changed_history.append(joints)
        self.view.publish_preview()

    # Called by Joint.save_setpoints. Needed for undo and redo.
    def save_changed_joints(self, joints):
        self.joint_changed_history.append(joints)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

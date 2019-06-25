import math
import os

import rospy
import rospkg

from urdf_parser_py import urdf
import pyqtgraph as pg

from pyqtgraph.Qt import QtCore, QtGui

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog, QPushButton, QFrame, \
    QLineEdit, QSlider, QHeaderView, QTableWidgetItem, QCheckBox, QMessageBox, QSpinBox, QDoubleSpinBox

import rviz

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

import GaitFactory
import UserInterfaceController

from model.Setpoint import Setpoint

from import_export import export_to_file, import_from_file_name

from JointSettingPlot import JointSettingPlot
from TimeSliderThread import TimeSliderThread


class GaitGeneratorPlugin(Plugin):
    DEFAULT_GAIT_DURATION = 8

    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)

        # Default values
        self.gait_publisher = None
        self.topic_name = ""
        self.gait_directory = None
        self.playback_speed = 100
        self.time_slider_thread = None
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.robot = urdf.Robot.from_parameter_server()
        self.gait = GaitFactory.empty_gait(self.robot, self.DEFAULT_GAIT_DURATION)

        # Start UI construction
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'gait_generator.ui')
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

        self.rviz_frame = self.create_rviz_frame()
        self._widget.RvizFrame.layout().addWidget(self.rviz_frame, 1, 0, 1, 3)

        # Store ui elements.
        self.change_gait_directory_button = self._widget.SettingsFrame.findChild(QPushButton, "ChangeGaitDirectory")
        self.import_gait_button = self._widget.SettingsFrame.findChild(QPushButton, "Import")
        self.export_gait_button = self._widget.SettingsFrame.findChild(QPushButton, "Export")
        self.publish_gait_button = self._widget.SettingsFrame.findChild(QPushButton, "Publish")

        # Connect Gait settings buttons
        self.set_gait_directory_button(self.gait_directory)
        self.change_gait_directory_button.clicked.connect(
            lambda: [
                self.set_gait_directory_button(self.get_gait_directory(True))
            ]
        )

        self.import_gait_button.clicked.connect(
            lambda: [
                self.load_gait()
            ]
        )

        self.export_gait_button.clicked.connect(self.export)

        self.publish_gait_button.clicked.connect(
            lambda: self.publish_gait()
        )

        self._widget.RvizFrame.findChild(QPushButton, "Start").clicked.connect(self.start_time_slider_thread)

        self._widget.RvizFrame.findChild(QPushButton, "Stop").clicked.connect(self.stop_time_slider_thread)

        self._widget.RvizFrame.findChild(QLineEdit, "PlaybackSpeed").setValidator(QtGui.QIntValidator(0, 500, self))
        self._widget.RvizFrame.findChild(QLineEdit, "PlaybackSpeed").editingFinished.connect(
            lambda: [
                self.set_playback_speed(float(self._widget.RvizFrame.findChild(QLineEdit, "PlaybackSpeed").text())),
                rospy.loginfo("Changing playbackspeed to " + str(self.playback_speed)),
            ]
        )

        self._widget.SettingsFrame.findChild(QLineEdit, "TopicName").editingFinished.connect(
            lambda: self.set_topic_name(self._widget.SettingsFrame.findChild(QLineEdit, "TopicName").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Gait").editingFinished.connect(
            lambda: self.gait.set_name(self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Gait").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Version").editingFinished.connect(
            lambda: self.gait.set_version(self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Version").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Subgait").editingFinished.connect(
            lambda: self.gait.set_subgait(self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Subgait").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Description").editingFinished.connect(
            lambda: self.gait.set_description(
                self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Description").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, "Duration").setKeyboardTracking(False)
        self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, "Duration").valueChanged.connect(
            lambda: self.update_gait_duration(
                self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, "Duration").value())
        )

        # Disable key inputs when mirroring is off.
        self._widget.SettingsFrame.findChild(QCheckBox, "Mirror").stateChanged.connect(
            lambda state: [
                self._widget.SettingsFrame.findChild(QLineEdit, "Key1").setEnabled(state),
                self._widget.SettingsFrame.findChild(QLineEdit, "Key2").setEnabled(state)
            ]
        )

        # Initialize the publisher on startup
        self.set_topic_name(self._widget.SettingsFrame.findChild(QLineEdit, "TopicName").text())

        self.load_gait_into_ui()

    def toggle_velocity_markers(self):
        self._widget.SettingsFrame.findChild(QCheckBox, "ShowVelocityMarkers").toggle()

    def create_rviz_frame(self):
        frame = rviz.VisualizationFrame()
        frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config,
                        os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'cfg.rviz'))
        frame.load(config)

        # Hide irrelevant Rviz details
        frame.setMenuBar(None)
        frame.setStatusBar(None)
        frame.setHideButtonVisibility(False)
        return frame

    def create_joint_settings(self):
        layout = self._widget.JointSettingContainer.layout()
        for i in reversed(range(layout.count())):
            widget = layout.itemAt(i).widget()
            layout.removeWidget(widget)
            widget.setParent(None)

        for i in range(0, len(self.robot.joints)):

            if self.robot.joints[i].type != "fixed":
                joint_name = self.robot.joints[i].name
                joint = self.gait.get_joint(joint_name)
                if not joint:
                    continue
                row = rospy.get_param("/joint_layout/" + joint_name + "/row", -1)
                column = rospy.get_param("/joint_layout/" + joint_name + "/column", -1)
                if row == -1 or column == -1:
                    rospy.logerr("Could not load the layout for joint %s. Please check config/layout.yaml", joint_name)
                    continue
                self._widget.JointSettingContainer.layout().addWidget(self.create_joint_setting(joint), row, column)

    def create_joint_setting(self, joint):
        joint_setting_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource',
                                          'joint_setting.ui')

        joint_setting = QFrame()
        loadUi(joint_setting_file, joint_setting)

        show_velocity_markers = self._widget.SettingsFrame.findChild(QCheckBox, "ShowVelocityMarkers").isChecked()
        joint_setting_plot = JointSettingPlot(joint, self.gait.duration, show_velocity_markers)

        self._widget.SettingsFrame.findChild(QCheckBox, "ShowVelocityMarkers").stateChanged.connect(
            lambda: [joint.set_setpoints(UserInterfaceController.plot_to_setpoints(joint_setting_plot)),
                     UserInterfaceController.update_ui_elements(
                         joint, table=joint_setting.Table, plot=joint_setting_plot, duration=self.gait.duration,
                         show_velocity_markers=self.show_velocity_markers_checked()),
                     self.publish_preview()
                     ]
        )

        # Connect a function to update the model and to update the table.
        joint_setting_plot.plot_item.sigPlotChanged.connect(
            lambda: [joint.set_setpoints(UserInterfaceController.plot_to_setpoints(joint_setting_plot)),
                     UserInterfaceController.update_ui_elements(
                         joint, table=joint_setting.Table, plot=joint_setting_plot, duration=self.gait.duration,
                         show_velocity_markers=self.show_velocity_markers_checked()),
                     self.publish_preview()
                     ])

        joint_setting_plot.add_setpoint.connect(
            lambda time, position, button: [
                self.add_setpoint(joint, time, position, button),
                UserInterfaceController.update_ui_elements(
                    joint, table=joint_setting.Table, plot=joint_setting_plot, duration=self.gait.duration,
                    show_velocity_markers=self.show_velocity_markers_checked()),
                self.publish_preview()
            ])

        joint_setting_plot.remove_setpoint.connect(
            lambda index: [
                joint.remove_setpoint(index),
                UserInterfaceController.update_ui_elements(
                    joint, table=joint_setting.Table, plot=joint_setting_plot, duration=self.gait.duration,
                    show_velocity_markers=self.show_velocity_markers_checked()),
                self.publish_preview()
            ])

        joint_setting.Plot.addItem(joint_setting_plot)

        joint_setting.Table = UserInterfaceController.update_table(
            joint_setting.Table, joint, self.gait.duration)

        joint_setting.Table.itemChanged.connect(
            lambda: [rospy.logwarn("item changed"),
                     joint.set_setpoints(UserInterfaceController.table_to_setpoints(joint_setting.Table)),
                     UserInterfaceController.update_ui_elements(
                         joint, plot=joint_setting_plot, duration=self.gait.duration,
                         show_velocity_markers=self.show_velocity_markers_checked()),
                     self.publish_preview()
                     ])

        # Disable scrolling horizontal
        joint_setting.Table.horizontalScrollBar().setDisabled(True)
        joint_setting.Table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        return joint_setting

    def show_velocity_markers_checked(self):
        return self._widget.SettingsFrame.findChild(QCheckBox, "ShowVelocityMarkers").isChecked()

    def add_setpoint(self, joint, time, position, button):
        if button == QtCore.Qt.ControlModifier:
            joint.add_interpolated_setpoint(time)
        else:
            joint.add_setpoint(Setpoint(time, position, 0))

    def get_gait_directory(self, select_new=False):
        if self.gait_directory is None or select_new:
            self.gait_directory = str(QFileDialog.getExistingDirectory(None, "Select a directory to save gaits"))
        if self.gait_directory == "":
            self.gait_directory = None
        rospy.loginfo("Selected output directory " + str(self.gait_directory))
        return self.gait_directory

    def set_gait_directory_button(self, gait_directory):
        if gait_directory is None:
            gait_directory = "Select a gait directory..."
        self.change_gait_directory_button.setText(gait_directory)

    def publish_preview(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.get_rostime()
        time = self.gait.current_time

        for i in range(len(self.gait.joints)):
            joint_state.name.append(self.gait.joints[i].name)
            joint_state.position.append(self.gait.joints[i].get_interpolated_position(time))
        self.joint_state_pub.publish(joint_state)

    def update_time_sliders(self):
        graphics_layouts = self._widget.JointSettingContainer.findChildren(pg.GraphicsLayoutWidget)
        for graphics_layout in graphics_layouts:
            joint_settings_plot = graphics_layout.getItem(0, 0)
            joint_settings_plot.updateTimeSlider(self.gait.current_time)

    def publish_gait(self):
        trajectory = self.gait.to_joint_trajectory()
        rospy.loginfo("Publishing trajectory to topic '" + self.topic_name + "'")
        self.gait_publisher.publish(trajectory)

    def set_topic_name(self, topic_name):
        self.topic_name = topic_name
        self.gait_publisher = rospy.Publisher(topic_name,
                                              JointTrajectory, queue_size=10)

    def set_playback_speed(self, playback_speed):
        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.playback_speed = playback_speed

        if was_playing:
            self.start_time_slider_thread()

    def start_time_slider_thread(self):
        if self.time_slider_thread is not None:
            rospy.logdebug("Cannot start another time slider thread as one is already active")
            return

        time_slider = self._widget.RvizFrame.findChild(QSlider, "TimeSlider")

        current = time_slider.value()
        playback_speed = self.playback_speed
        max = time_slider.maximum()
        self.time_slider_thread = TimeSliderThread(current, playback_speed, max)
        self.time_slider_thread.update_signal.connect(self.update_main_time_slider)
        self.time_slider_thread.start()

    def update_gait_duration(self, duration):
        rescale_setpoints = self._widget.GaitPropertiesFrame.findChild(QCheckBox, "ScaleSetpoints").isChecked()

        if self.gait.has_setpoints_after_duration(duration) and not rescale_setpoints:
            if not self.gait.has_multiple_setpoints_before_duration(duration):
                QMessageBox.question(self._widget, 'Could not update gait duration',
                                     "Not all joints have multiple setpoints before duration " + str(duration),
                                     QMessageBox.Ok)
                return
            discard_setpoints = QMessageBox.question(self._widget, 'Gait duration lower than highest time setpoint',
                                                     "Do you want to discard any setpoints higher than the given "
                                                     "duration?",
                                                     QMessageBox.Yes | QMessageBox.No)
            if discard_setpoints == QMessageBox.No:
                self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, "Duration").setValue(self.gait.duration)
                return
        self.gait.set_duration(duration, rescale_setpoints)
        self._widget.RvizFrame.findChild(QSlider, "TimeSlider").setRange(0, 100 * self.gait.duration)

        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.create_joint_settings()

        if was_playing:
            self.start_time_slider_thread()

    def stop_time_slider_thread(self):
        if self.time_slider_thread is not None:
            self.time_slider_thread.stop()
            self.time_slider_thread = None

    def export(self):
        should_mirror = self._widget.SettingsFrame.findChild(QCheckBox, "Mirror").isChecked()

        key_1 = self._widget.SettingsFrame.findChild(QLineEdit, "Key1").text()
        key_2 = self._widget.SettingsFrame.findChild(QLineEdit, "Key2").text()

        if should_mirror:
            mirror = self.gait.get_mirror(key_1, key_2)
            if mirror:
                export_to_file(mirror, self.get_gait_directory())
            else:
                UserInterfaceController.notify("Could not mirror gait", "Check the logs for more information.")
                return

        export_to_file(self.gait, self.get_gait_directory()),
        self.set_gait_directory_button(self.gait_directory)

    def toggle_time_slider_thread(self):
        if self.thread is None:
            self.start_time_slider_thread()
        else:
            self.stop_time_slider_thread()

    def load_gait(self):

        file_name, f = QFileDialog.getOpenFileName(self._widget,
                                                   "Open Image",
                                                   rospkg.RosPack().get_path('march_rqt_gait_generator'),
                                                   "March Subgait (*.subgait)")

        gait_directory = '/'.join(file_name.split('/')[:-3])
        if gait_directory != "":
            rospy.loginfo("Setting gait directory to %s", str(gait_directory))
            self.gait_directory = gait_directory
            self.set_gait_directory_button(gait_directory)

        self.gait = import_from_file_name(self.robot, file_name)
        if self.gait is None:
            rospy.logwarn("Could not load gait %s", file_name)
        else:
            self.load_gait_into_ui()

    def load_gait_into_ui(self):
        time_slider = self._widget.RvizFrame.findChild(QSlider, "TimeSlider")
        time_slider.setRange(0, 100 * self.gait.duration)

        # Connect TimeSlider to the preview
        time_slider.valueChanged.connect(lambda: [
            self.gait.set_current_time(float(time_slider.value()) / 100),
            self.publish_preview(),
            self.update_time_sliders(),
        ])

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Gait").setText(self.gait.name)
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Subgait").setText(self.gait.subgait)
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Version").setText(self.gait.version)
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Description").setText(self.gait.description)

        # Block signals on the duration edit to prevent a reload of the joint settings
        self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, "Duration").blockSignals(True)
        self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, "Duration").setValue(self.gait.duration)
        self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, "Duration").blockSignals(False)

        print ('load gait into ui')
        self.create_joint_settings()

        self.publish_preview()

    @QtCore.pyqtSlot(int)
    def update_main_time_slider(self, time):
        self._widget.RvizFrame.findChild(QSlider, "TimeSlider").setValue(time)

    def shutdown_plugin(self):
        self.stop_time_slider_thread()

    def save_settings(self, plugin_settings, instance_settings):
        plugin_settings.set_value('gait_directory', self.gait_directory)

    def restore_settings(self, plugin_settings, instance_settings):
        gait_directory = plugin_settings.value('gait_directory')

        if gait_directory is not None:
            rospy.loginfo("Restoring saved gait directory " + str(gait_directory))
            self.gait_directory = gait_directory

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

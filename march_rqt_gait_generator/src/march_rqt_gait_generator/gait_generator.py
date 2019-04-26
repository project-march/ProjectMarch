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
    QLineEdit, QSlider, QHeaderView, QTableWidgetItem, QCheckBox, QMessageBox

import rviz

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

import GaitFactory
import UserInterfaceController

from model.Setpoint import Setpoint
from model.Joint import Joint
from model.Limits import Limits
from model.Gait import Gait

from import_export import export_to_file

from JointSettingPlot import JointSettingPlot
from TimeSliderThread import TimeSliderThread


class GaitGeneratorPlugin(Plugin):
    DEFAULT_GAIT_DURATION = 12

    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)

        # Default values
        self.gait_publisher = None
        self.topic_name = ""
        self.gait_directory = None
        self.playback_speed = 100
        self.thread = None
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.robot = urdf.Robot.from_parameter_server()
        self.gait = GaitFactory.empty_gait(self.robot, 10)

        # History variable to avoid a Qt bug when the gait duration changes.
        self.last_duration = self.gait.duration

        # Start UI construction
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'gait_generator.ui')
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

        self.rviz_frame = self.create_rviz_frame()
        self._widget.RvizFrame.layout().addWidget(self.rviz_frame, 1, 0, 1, 3)

        time_slider = self._widget.RvizFrame.findChild(QSlider, "TimeSlider")
        time_slider.setRange(0, 100 * self.gait.duration)

        # Connect TimeSlider to the preview
        time_slider.valueChanged.connect(lambda: [
            self.gait.set_current_time(float(time_slider.value()) / 100),
            self.publish_preview(),
            self.update_time_sliders(),
        ])

        # Connect Gait settings buttons
        self._widget.SettingsFrame.findChild(QPushButton, "Export").clicked.connect(
            lambda: [
                self.get_gait_directory(),
                export_to_file(self.gait, self.gait_directory)
            ]
        )

        self._widget.SettingsFrame.findChild(QPushButton, "Publish").clicked.connect(
            lambda: self.publish_gait()
        )

        self._widget.RvizFrame.findChild(QPushButton, "Start").clicked.connect(self.start_time_slider_thread)

        self._widget.RvizFrame.findChild(QPushButton, "Stop").clicked.connect(self.stop_time_slider_thread)

        self._widget.RvizFrame.findChild(QLineEdit, "PlaybackSpeed").setValidator(QtGui.QIntValidator(0, 500, self))
        self._widget.RvizFrame.findChild(QLineEdit, "PlaybackSpeed").editingFinished.connect(
            lambda: [
                self.stop_time_slider_thread(),
                self.set_playback_speed(float(self._widget.RvizFrame.findChild(QLineEdit, "PlaybackSpeed").text())),
                rospy.loginfo("Changing playbackspeed to " + str(self.playback_speed)),
                self.start_time_slider_thread()
            ]
        )

        self._widget.SettingsFrame.findChild(QLineEdit, "TopicName").editingFinished.connect(
            lambda: self.set_topic_name(self._widget.SettingsFrame.findChild(QLineEdit, "TopicName").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Name").setText(self.gait.name)
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Name").editingFinished.connect(
            lambda: self.gait.set_name(self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Name").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Version").setText(self.gait.version)
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Version").editingFinished.connect(
            lambda: self.gait.set_version(self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Version").text())
        )
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Description").setText(self.gait.description)
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Description").editingFinished.connect(
            lambda: self.gait.set_description(
                self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Description").text())
        )

        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Duration").setValidator(
            QtGui.QDoubleValidator(1.0, 20.0, QtGui.QDoubleValidator.StandardNotation, self))
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Duration").setText(str(self.gait.duration))
        self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Duration").returnPressed.connect(
            lambda: self.update_gait_duration(
                float(self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Duration").text()))
        )

        # Initialize the publisher on startup
        self.set_topic_name(self._widget.SettingsFrame.findChild(QLineEdit, "TopicName").text())

        self.create_joint_settings()

        self.publish_preview()

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
            widgetToRemove = layout.itemAt(i).widget()
            # remove it from the layout list
            layout.removeWidget(widgetToRemove)
            # remove it from the gui
            widgetToRemove.setParent(None)

        for i in range(0, len(self.gait.joints)):
            self._widget.JointSettingContainer.layout().addWidget(self.create_joint_setting(self.gait.joints[i]), i % 3,
                                                                  i >= 3)

    def create_joint_setting(self, joint):
        joint_setting_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource',
                                          'joint_setting.ui')

        joint_setting = QFrame()
        loadUi(joint_setting_file, joint_setting)

        joint_setting_plot = JointSettingPlot(joint, self.gait.duration)

        # Connect a function to update the model and to update the table.
        joint_setting_plot.plot_item.sigPlotChanged.connect(
            lambda: [joint.set_setpoints(UserInterfaceController.plot_to_setpoints(joint_setting_plot)),
                     UserInterfaceController.update_ui_elements(
                         joint, table=joint_setting.Table, plot=joint_setting_plot),
                     self.publish_preview()
                     ])

        joint_setting_plot.add_setpoint.connect(
            lambda time, position, button: [
                self.add_setpoint(joint, time, position, button),
                UserInterfaceController.update_ui_elements(joint, table=joint_setting.Table, plot=joint_setting_plot),
                self.publish_preview()
            ])

        joint_setting_plot.remove_setpoint.connect(
            lambda index: [
                joint.remove_setpoint(index),
                UserInterfaceController.update_ui_elements(joint, table=joint_setting.Table, plot=joint_setting_plot),
                self.publish_preview()
            ])

        joint_setting.Plot.addItem(joint_setting_plot)

        joint_setting.Table = UserInterfaceController.update_table(joint_setting.Table, joint.setpoints)

        # Todo(Isha) refactor to check if new item is valid and don't update if invalid.
        joint_setting.Table.itemChanged.connect(
            lambda: [joint.set_setpoints(UserInterfaceController.table_to_setpoints(joint_setting.Table)),
                     UserInterfaceController.update_ui_elements(
                         joint, table=joint_setting.Table, plot=joint_setting_plot),
                     self.publish_preview()
                     ])

        # Disable scrolling vertically
        joint_setting.Table.verticalScrollBar().setDisabled(True)
        joint_setting.Table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)

        return joint_setting

    def add_setpoint(self, joint, time, position, button):
        if button == QtCore.Qt.ControlModifier:
            joint.add_interpolated_setpoint(time)
        else:
            joint.add_setpoint(Setpoint(time, position, 0))

    def get_gait_directory(self):
        if self.gait_directory is None:
            self.gait_directory = str(QFileDialog.getExistingDirectory(None, "Select a directory to save gaits"))
        return self.gait_directory

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
        self.playback_speed = playback_speed

    def start_time_slider_thread(self):
        time_slider = self._widget.RvizFrame.findChild(QSlider, "TimeSlider")

        current = time_slider.value()
        playback_speed = self.playback_speed
        max = time_slider.maximum()
        self.thread = TimeSliderThread(current, playback_speed, max)
        self.thread.update_signal.connect(self.update_main_time_slider)
        self.thread.start()

    def update_gait_duration(self, duration):

        # Workaround to prevent from updating twice.
        if duration == self.last_duration:
            return

        self.last_duration = duration

        rescale_setpoints = self._widget.GaitPropertiesFrame.findChild(QCheckBox, "ScaleSetpoints").isChecked()

        if self.gait.has_setpoints_after_duration(duration) and not rescale_setpoints:

            discard_setpoints = QMessageBox.question(self._widget, 'Gait duration lower than highest time setpoint',
                                                     "Do you want to discard any setpoints higher than the given "
                                                     "duration?",
                                                     QMessageBox.Yes | QMessageBox.No)
            if discard_setpoints == QMessageBox.No:
                self.last_duration = None
                self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Duration").setText(str(self.gait.duration))
                return
        self.gait.set_duration(duration, rescale_setpoints)
        self._widget.RvizFrame.findChild(QSlider, "TimeSlider").setRange(0, 100 * self.gait.duration)

        self.stop_time_slider_thread()
        self.create_joint_settings()
        self.start_time_slider_thread()


    def stop_time_slider_thread(self):
        if self.thread is not None:
            self.thread.stop()

    @QtCore.pyqtSlot(str)
    def update_main_time_slider(self, time):
        self._widget.RvizFrame.findChild(QSlider, "TimeSlider").setValue(time)

    def shutdown_plugin(self):
        self.stop_time_slider_thread()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        plugin_settings.set_value('test', 3)
        v = plugin_settings.value('test')
        rospy.loginfo(v)

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        v = plugin_settings.value('test')
        rospy.loginfo(v)

    # def trigger_configuration(self):
    # Comment in to signal that the plugin has a way to configure
    # This will enable a setting button (gear icon) in each dock widget title bar
    # Usually used to open a modal configuration dialog

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


class GaitGeneratorPlugin(Plugin):

    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)

        # Default values
        self.gait_publisher = None
        self.topic_name = ''
        self.gait_directory = None
        self.playback_speed = 100
        self.time_slider_thread = None
        self.current_time = 0
        self.tf_listener = TransformListener()
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joint_changed_history = RingBuffer(capacity=100, dtype=list)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

        self.robot = urdf.Robot.from_parameter_server()
        self.gait = ModifiableSubgait.empty_subgait(self, self.robot)

        self.build_ui(context)
        self.set_topic_name(self.topic_name_line_edit.text())
        self.load_gait_into_ui()

    # Called by __init__
    def build_ui(self, context):
        # Start UI construction
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'gait_generator.ui')
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

        self.rviz_frame = self.create_rviz_frame()
        self._widget.RvizFrame.layout().addWidget(self.rviz_frame, 1, 0, 1, 3)

        # Store ui elements.
        self.change_gait_directory_button = self._widget.SettingsFrame.findChild(QPushButton, 'ChangeGaitDirectory')
        self.import_gait_button = self._widget.SettingsFrame.findChild(QPushButton, 'Import')
        self.export_gait_button = self._widget.SettingsFrame.findChild(QPushButton, 'Export')
        self.publish_gait_button = self._widget.SettingsFrame.findChild(QPushButton, 'Publish')
        self.start_button = self._widget.RvizFrame.findChild(QPushButton, 'Start')
        self.stop_button = self._widget.RvizFrame.findChild(QPushButton, 'Stop')
        self.invert_button = self._widget.RvizFrame.findChild(QPushButton, 'Invert')
        self.undo_button = self._widget.RvizFrame.findChild(QPushButton, 'Undo')
        self.redo_button = self._widget.RvizFrame.findChild(QPushButton, 'Redo')
        self.playback_speed_spin_box = self._widget.RvizFrame.findChild(QSpinBox, 'PlaybackSpeed')
        self.height_left_line_edit = self._widget.RvizFrame.findChild(QLineEdit, 'HeightLeft')
        self.height_right_line_edit = self._widget.RvizFrame.findChild(QLineEdit, 'HeightRight')
        self.heel_distance_line_edit = self._widget.RvizFrame.findChild(QLineEdit, 'HeelHeelDistance')
        self.topic_name_line_edit = self._widget.SettingsFrame.findChild(QLineEdit, 'TopicName')
        self.gait_name_line_edit = self._widget.GaitPropertiesFrame.findChild(QLineEdit, 'Gait')
        self.subgait_name_line_edit = self._widget.GaitPropertiesFrame.findChild(QLineEdit, 'Subgait')
        self.version_name_line_edit = self._widget.GaitPropertiesFrame.findChild(QLineEdit, 'Version')
        self.description_line_edit = self._widget.GaitPropertiesFrame.findChild(QLineEdit, 'Description')
        self.gait_type_combo_box = self._widget.GaitPropertiesFrame.findChild(QComboBox, 'GaitType')
        self.gait_type_combo_box.addItems(['walk_like', 'sit_like', 'stairs_like'])
        self.duration_spin_box = self._widget.GaitPropertiesFrame.findChild(QDoubleSpinBox, 'Duration')
        self.mirror_check_box = self._widget.SettingsFrame.findChild(QCheckBox, 'Mirror')
        self.mirror_key1_line_edit = self._widget.SettingsFrame.findChild(QLineEdit, 'Key1')
        self.mirror_key2_line_edit = self._widget.SettingsFrame.findChild(QLineEdit, 'Key2')
        self.velocity_markers_check_box = self._widget.SettingsFrame.findChild(QCheckBox, 'ShowVelocityMarkers')
        self.time_slider = self._widget.RvizFrame.findChild(QSlider, 'TimeSlider')
        self.scale_setpoints_check_box = self._widget.GaitPropertiesFrame.findChild(QCheckBox, 'ScaleSetpoints')

        self.connect_buttons()

    # Called by build_ui
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

    # Called by build_ui
    def connect_buttons(self):
        self.change_gait_directory_button.clicked.connect(self.change_gait_directory)
        self.import_gait_button.clicked.connect(self.import_gait)
        self.export_gait_button.clicked.connect(self.export_gait)

        self.publish_gait_button.clicked.connect(self.publish_gait)

        self.start_button.clicked.connect(self.start_time_slider_thread)
        self.stop_button.clicked.connect(self.stop_time_slider_thread)
        self.invert_button.clicked.connect(self.invert_gait)
        self.undo_button.clicked.connect(self.undo)
        self.redo_button.clicked.connect(self.redo)

        # Line edits / combo boxes / spin boxes
        self.gait_type_combo_box.currentTextChanged.connect(
            lambda text: self.gait.set_gait_type(text),
        )
        self.gait_name_line_edit.textChanged.connect(
            lambda text: self.gait.set_gait_name(text),
        )
        self.version_name_line_edit.textChanged.connect(
            lambda text: self.gait.set_version(text),
        )
        self.subgait_name_line_edit.textChanged.connect(
            lambda text: self.gait.set_subgait_name(text),
        )
        self.description_line_edit.textChanged.connect(
            lambda text: self.gait.set_description(text),
        )
        self.topic_name_line_edit.textChanged.connect(self.set_topic_name)
        self.playback_speed_spin_box.valueChanged.connect(self.set_playback_speed)
        self.duration_spin_box.setKeyboardTracking(False)
        self.duration_spin_box.valueChanged.connect(self.update_gait_duration)

        # Disable key inputs when mirroring is off.
        self.mirror_check_box.stateChanged.connect(
            lambda state: [
                self.mirror_key1_line_edit.setEnabled(state),
                self.mirror_key2_line_edit.setEnabled(state),
            ],
        )

    # Called by __init__ and import_gait.
    def load_gait_into_ui(self):
        self.time_slider.setRange(0, 100 * self.gait.duration)

        # Connect TimeSlider to the preview
        self.time_slider.valueChanged.connect(lambda: [
            self.set_current_time(float(self.time_slider.value()) / 100),
            self.publish_preview(),
            self.update_time_sliders(),
        ])

        self.gait_type_combo_box.setCurrentText(self.gait.gait_type)
        self.gait_name_line_edit.setText(self.gait.gait_name)
        self.subgait_name_line_edit.setText(self.gait.subgait_name)
        self.version_name_line_edit.setText(self.gait.version)
        self.description_line_edit.setText(self.gait.description)

        # Block signals on the duration edit to prevent a reload of the joint settings
        self.duration_spin_box.blockSignals(True)
        self.duration_spin_box.setValue(self.gait.duration)
        self.duration_spin_box.blockSignals(False)

        self.create_joint_settings()
        self.publish_preview()

    # Called by load_gait_into_ui.
    def update_time_sliders(self):
        graphics_layouts = self._widget.JointSettingContainer.findChildren(pg.GraphicsLayoutWidget)
        for graphics_layout in graphics_layouts:
            joint_settings_plot = graphics_layout.getItem(0, 0)
            joint_settings_plot.update_time_slider(self.current_time)

    # Called by load_gait_into_ui and update_gait_duration.
    def create_joint_settings(self):
        layout = self._widget.JointSettingContainer.layout()
        for i in reversed(range(layout.count())):
            widget = layout.itemAt(i).widget()
            layout.removeWidget(widget)
            widget.setParent(None)

        for i in range(0, len(self.robot.joints)):

            if self.robot.joints[i].type != 'fixed':
                joint_name = self.robot.joints[i].name
                joint = self.gait.get_joint(joint_name)
                if not joint:
                    continue
                row = rospy.get_param('/joint_layout/' + joint_name + '/row', -1)
                column = rospy.get_param('/joint_layout/' + joint_name + '/column', -1)
                if row == -1 or column == -1:
                    rospy.logerr('Could not load the layout for joint %s. Please check config/layout.yaml', joint_name)
                    continue
                self._widget.JointSettingContainer.layout().addWidget(self.create_joint_setting(joint), row, column)

    # Called 8 times by create_joint_settings.
    def create_joint_setting(self, joint):
        joint_setting_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource',
                                          'joint_setting.ui')

        joint_setting = QFrame()
        loadUi(joint_setting_file, joint_setting)

        show_velocity_markers = self.velocity_markers_check_box.isChecked()
        joint_setting_plot = JointSettingPlot(joint, self.gait.duration, show_velocity_markers)
        joint_setting.Plot.addItem(joint_setting_plot)

        joint_setting.Table = user_interface_controller.update_table(
            joint_setting.Table, joint, self.gait.duration)
        # Disable scrolling horizontally
        joint_setting.Table.horizontalScrollBar().setDisabled(True)
        joint_setting.Table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        def update_joint_ui():
            user_interface_controller.update_ui_elements(
                joint, table=joint_setting.Table, plot=joint_setting_plot, duration=self.gait.duration,
                show_velocity_markers=self.velocity_markers_check_box.isChecked())
            self.publish_preview()

        def add_setpoint(joint, time, position, button):
            if button == QtCore.Qt.ControlModifier:
                joint.add_interpolated_setpoint(time)
            else:
                joint.add_setpoint(ModifiableSetpoint(time, position, 0))

        self.undo_button.clicked.connect(update_joint_ui)
        self.redo_button.clicked.connect(update_joint_ui)
        self.invert_button.clicked.connect(update_joint_ui)

        self.velocity_markers_check_box.stateChanged.connect(
            lambda: [
                joint.set_setpoints(user_interface_controller.plot_to_setpoints(joint_setting_plot)),
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
                    show_velocity_markers=self.velocity_markers_check_box.isChecked()),
                self.publish_preview(),
            ])

        return joint_setting

    # Functions below are connected to buttons, text boxes, joint graphs etc.

    def publish_preview(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.get_rostime()
        time = self.current_time

        for i in range(len(self.gait.joints)):
            joint_state.name.append(self.gait.joints[i].name)
            joint_state.position.append(self.gait.joints[i].get_interpolated_position(time))
        self.joint_state_pub.publish(joint_state)
        self.set_feet_distances()

    def toggle_velocity_markers(self):
        self.velocity_markers_check_box.toggle()

    def publish_gait(self):
        trajectory = self.gait._to_joint_trajectory_msg()
        rospy.loginfo('Publishing trajectory to topic ' + self.topic_name + '')
        self.gait_publisher.publish(trajectory)

    def set_current_time(self, current_time):
        self.current_time = current_time

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

        current = self.time_slider.value()
        playback_speed = self.playback_speed
        max_time = self.time_slider.maximum()
        self.time_slider_thread = TimeSliderThread(current, playback_speed, max_time)
        self.time_slider_thread.update_signal.connect(self.update_main_time_slider)
        self.time_slider_thread.start()

    def stop_time_slider_thread(self):
        if self.time_slider_thread is not None:
            self.time_slider_thread.stop()
            self.time_slider_thread = None

    def update_gait_duration(self, duration):
        rescale_setpoints = self.scale_setpoints_check_box.isChecked()

        if self.gait.has_setpoints_after_duration(duration) and not rescale_setpoints:
            if not self.gait.has_multiple_setpoints_before_duration(duration):
                QMessageBox.question(self._widget, 'Could not update gait duration',
                                     'Not all joints have multiple setpoints before duration ' + str(duration),
                                     QMessageBox.Ok)
                return
            discard_setpoints = QMessageBox.question(self._widget, 'Gait duration lower than highest time setpoint',
                                                     'Do you want to discard any setpoints higher than the given '
                                                     'duration?',
                                                     QMessageBox.Yes | QMessageBox.No)
            if discard_setpoints == QMessageBox.No:
                self.duration_spin_box.setValue(self.gait.duration)
                return
        self.gait.set_duration(duration, rescale_setpoints)
        self.time_slider.setRange(0, 100 * self.gait.duration)

        was_playing = self.time_slider_thread is not None
        self.stop_time_slider_thread()

        self.create_joint_settings()

        if was_playing:
            self.start_time_slider_thread()

    def import_gait(self):
        file_name, f = QFileDialog.getOpenFileName(self._widget,
                                                   'Open Image',
                                                   os.getenv('HOME') + '/march_ws/src/gait-files/march_gait_files',
                                                   'March Subgait (*.subgait)')

        gait = ModifiableSubgait.from_file(self, self.robot, file_name)
        if gait is None:
            rospy.logwarn('Could not load gait %s', file_name)
            return
        self.gait = gait
        self.load_gait_into_ui()

        self.gait_directory = '/'.join(file_name.split('/')[:-3])
        rospy.loginfo('Setting gait directory to %s', str(self.gait_directory))
        self.change_gait_directory_button.setText(self.gait_directory)

        # Clear undo and redo history
        self.joint_changed_history = RingBuffer(capacity=100, dtype=list)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

    def export_gait(self):
        should_mirror = self.mirror_check_box.isChecked()

        key_1 = self.mirror_key1_line_edit.text()
        key_2 = self.mirror_key2_line_edit.text()

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
            self.change_gait_directory_button.setText('Select a gait directory...')
        else:
            self.change_gait_directory_button.setText(self.gait_directory)

    def invert_gait(self):
        for joint in self.gait.joints:
            joint.invert()
        self.save_changed_joints(self.gait.joints)
        self.publish_preview()

    def undo(self):
        if not self.joint_changed_history:
            return

        joints = self.joint_changed_history.pop()
        for joint in joints:
            joint.undo()
        self.joint_changed_redo_list.append(joints)
        self.publish_preview()

    def redo(self):
        if not self.joint_changed_redo_list:
            return

        joints = self.joint_changed_redo_list.pop()
        for joint in joints:
            joint.redo()
        self.joint_changed_history.append(joints)
        self.publish_preview()

    # Called by Joint.save_setpoints. Needed for undo and redo.
    def save_changed_joints(self, joints):
        self.joint_changed_history.append(joints)
        self.joint_changed_redo_list = RingBuffer(capacity=100, dtype=list)

    # Called to update values in Heigt left foot etc.
    def set_feet_distances(self):
        try:
            # The translation from the right foot to the left foot is the position of the
            # left foot from the right foot's frame of reference.
            (trans_left, rot_right) = self.tf_listener.lookupTransform('/foot_right', '/foot_left', rospy.Time(0))
            (trans_right, rot_left) = self.tf_listener.lookupTransform('/foot_left', '/foot_right', rospy.Time(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        self.height_left_line_edit.setText('%.3f' % trans_left[2])
        self.height_right_line_edit.setText('%.3f' % trans_right[2])
        self.heel_distance_line_edit.setText('%.3f' % math.sqrt(trans_left[0] ** 2 + trans_left[2] ** 2))

    @QtCore.pyqtSlot(int)
    def update_main_time_slider(self, time):
        self.time_slider.setValue(time)

    def shutdown_plugin(self):
        self.stop_time_slider_thread()

    def save_settings(self, plugin_settings, instance_settings):
        plugin_settings.set_value('gait_directory', self.gait_directory)

    def restore_settings(self, plugin_settings, instance_settings):
        gait_directory = plugin_settings.value('gait_directory')

        if gait_directory is not None:
            rospy.loginfo('Restoring saved gait directory ' + str(gait_directory))
            self.gait_directory = gait_directory

        # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

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
from .gait_generator_controller import GaitGeneratorController


class GaitGeneratorView(Plugin):

    def __init__(self, context):
        super(GaitGeneratorView, self).__init__(context)

        self.current_time = 0
        self.tf_listener = TransformListener()
        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.build_ui(context)

        self.controller = GaitGeneratorController(self)
        self.load_gait_into_ui(self.controller.gait)

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
        self.velocity_plot_check_box = self._widget.SettingsFrame.findChild(QCheckBox, 'ShowVelocityPlot')
        self.effort_plot_check_box = self._widget.SettingsFrame.findChild(QCheckBox, 'ShowEffortPlot')
        self.time_slider = self._widget.RvizFrame.findChild(QSlider, 'TimeSlider')
        self.scale_setpoints_check_box = self._widget.GaitPropertiesFrame.findChild(QCheckBox, 'ScaleSetpoints')

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

    # Called by __init__ and import_gait.
    def load_gait_into_ui(self, gait):
        self.time_slider.setRange(0, 100 * gait.duration)

        # Connect TimeSlider to the preview
        self.time_slider.valueChanged.connect(lambda: [
            self.set_current_time(float(self.time_slider.value()) / 100),
            self.publish_preview(),
            self.update_time_sliders(),
        ])

        self.gait_type_combo_box.setCurrentText(gait.gait_type)
        self.gait_name_line_edit.setText(gait.gait_name)
        self.subgait_name_line_edit.setText(gait.subgait_name)
        self.version_name_line_edit.setText(gait.version)
        self.description_line_edit.setText(gait.description)

        # Block signals on the duration edit to prevent a reload of the joint settings
        self.duration_spin_box.blockSignals(True)
        self.duration_spin_box.setValue(gait.duration)
        self.duration_spin_box.blockSignals(False)

        self.build_joint_plots()
        self.publish_preview()

    # Called by load_gait_into_ui.
    def update_time_sliders(self):
        graphics_layouts = self._widget.JointSettingContainer.findChildren(pg.GraphicsLayoutWidget)
        for graphics_layout in graphics_layouts:
            joint_settings_plot = graphics_layout.getItem(0, 0)
            joint_settings_plot.update_time_slider(self.current_time)

    # Called by load_gait_into_ui and update_gait_duration.
    def build_joint_plots(self):
        layout = self._widget.JointSettingContainer.layout()
        for i in reversed(range(layout.count())):
            widget = layout.itemAt(i).widget()
            layout.removeWidget(widget)
            widget.setParent(None)

        for joint_name, joint_plot in self.controller.create_joint_settings():
                row = rospy.get_param('/joint_layout/' + joint_name + '/row', -1)
                column = rospy.get_param('/joint_layout/' + joint_name + '/column', -1)
                if row == -1 or column == -1:
                    rospy.logerr('Could not load the layout for joint %s. Please check config/layout.yaml', joint_name)
                    continue
                self._widget.JointSettingContainer.layout().addWidget(joint_plot, row, column)

    # Functions below are connected to buttons, text boxes, joint graphs etc.
    def publish_preview(self):
        joint_state = JointState()
        joint_state.header.stamp = rospy.get_rostime()
        time = self.current_time

        for i in range(len(self.controller.gait.joints)):
            joint_state.name.append(self.controller.gait.joints[i].name)
            joint_state.position.append(self.controller.gait.joints[i].get_interpolated_position(time))
        self.joint_state_pub.publish(joint_state)
        self.set_feet_distances()

    # Called to update values in Heigt left foot etc.
    def set_current_time(self, current_time):
        self.current_time = current_time

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
        self.controller.stop_time_slider_thread()

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

    def message(self, title=None, msg=None):
        QMessageBox.question(self._widget, title, msg, QMessageBox.Ok)

    def yes_no_question(self, title=None, msg=None):
        answer = QMessageBox.question(self._widget, title, msg, QMessageBox.Yes | QMessageBox.No)
        return answer == QMessageBox.Yes

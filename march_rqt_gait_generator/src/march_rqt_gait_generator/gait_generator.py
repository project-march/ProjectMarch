import math
import os

import rospy
import rospkg

from urdf_parser_py import urdf
import pyqtgraph as pg

from pyqtgraph.Qt import QtCore

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QFileDialog
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QFrame
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QHeaderView
from python_qt_binding.QtWidgets import QTableWidgetItem

import rviz

from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

from march_rqt_gait_generator.JointSettingPlot import JointSettingPlot
from march_rqt_gait_generator.model.Setpoint import Setpoint
from march_rqt_gait_generator.model.Joint import Joint
from march_rqt_gait_generator.model.Limits import Limits
from march_rqt_gait_generator.model.Gait import Gait


class GaitGeneratorPlugin(Plugin):
    TABLE_DIGITS = 4
    DEFAULT_GAIT_DURATION = 12

    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)
        self.setObjectName('GaitGeneratorPlugin')

        self.joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        self.robot = urdf.Robot.from_parameter_server()
        self.gait = self.create_empty_gait()
        self.gait_publisher = None
        self.topic_name = ""
        self.gait_directory = None

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        self._widget = QWidget()

        # Load main UI file
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'gait_generator.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('Gait Generator')

        # Add widget to the user interface
        context.add_widget(self._widget)

        # Load and configure Rviz
        self.frame = rviz.VisualizationFrame()
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config,
                        os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'cfg.rviz'))
        self.frame.load(config)

        # Hide irrelevant Rviz details
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self._widget.RvizFrame.layout().addWidget(self.frame, 1, 0)

        time_slider = self._widget.RvizFrame.findChild(QSlider, "TimeSlider")
        time_slider.setRange(0, 1000)

        # Connect TimeSlider to the preview
        time_slider.valueChanged.connect(lambda: [
            self.gait.set_current_time(float(time_slider.value() / 1000.0) * self.gait.duration),
            self.publish_preview(),
            self.update_time_sliders(),
        ])

        # Connect Gait settings buttons
        self._widget.SettingsFrame.findChild(QPushButton, "ExportButton").clicked.connect(
            lambda: self.export_to_file()
        )

        self._widget.SettingsFrame.findChild(QPushButton, "PublishButton").clicked.connect(
            lambda: self.publish_gait()
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
            lambda: self.gait.set_description(self._widget.GaitPropertiesFrame.findChild(QLineEdit, "Description").text())
        )

    # Initialize the publisher on startup
        self.set_topic_name(self._widget.SettingsFrame.findChild(QLineEdit, "TopicName").text())

        self.create_joint_settings()

        self.publish_preview()

    def create_empty_gait(self):
        if self.robot is None:
            rospy.logerr("Cannot create gait without a loaded robot.")
        joint_list = []
        for i in range(0, len(self.robot.joints)):
            urdf_joint = self.robot.joints[i]
            if urdf_joint.type == "fixed":
                rospy.loginfo("Skipping fixed joint " + urdf_joint.name)
                continue

            if urdf_joint.limit is None:
                rospy.logwarn("Skipping joint " + urdf_joint.name + " because it has no limits.")
                continue

            default_setpoints = [
                Setpoint(0.2, 0, 0),
                Setpoint(3, 1.3, 0),
                Setpoint(4, 1.3, 0),
                Setpoint(self.DEFAULT_GAIT_DURATION, 0, 0)
            ]
            joint = Joint(urdf_joint.name,
                          Limits(urdf_joint.limit.upper, urdf_joint.limit.lower, urdf_joint.limit.velocity),
                          default_setpoints,
                          self.DEFAULT_GAIT_DURATION
                          )
            joint_list.append(joint)
        return Gait(joint_list, self.DEFAULT_GAIT_DURATION)

    def create_joint_settings(self):
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
            lambda: [joint.set_setpoints(self.plot_to_setpoints(joint_setting_plot)),
                     self.update_ui_elements(joint, table=joint_setting.Table, plot=joint_setting_plot),
                     ])

        joint_setting_plot.add_setpoint.connect(
            lambda time, position, button: [
                self.add_setpoint(joint, time, position, button),
                self.update_ui_elements(joint, table=joint_setting.Table, plot=joint_setting_plot)
            ])

        joint_setting_plot.remove_setpoint.connect(
            lambda index: [
                joint.remove_setpoint(index),
                self.update_ui_elements(joint, table=joint_setting.Table, plot=joint_setting_plot)
            ])

        joint_setting.Plot.addItem(joint_setting_plot)

        # Populate table with data and resize
        joint_setting.Table = self.update_table(joint_setting.Table, joint.setpoints)

        # Disconnect the signals on the plot to avoid an infinite loop of table to plot to table updates.
        # Todo(Isha) refactor to check if new item is valid and don't update if invalid.
        joint_setting.Table.itemChanged.connect(
            lambda: [joint.set_setpoints(self.table_to_setpoints(joint_setting.Table)),
                     self.update_ui_elements(joint, table=joint_setting.Table, plot=joint_setting_plot)
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

    def plot_to_setpoints(self, plot):
        plot_data = plot.plot_item.getData()
        setpoints = []
        for i in range(0, len(plot_data[0])):
            velocity = plot.velocities[i]
            time = plot_data[0][i]
            position = math.radians(plot_data[1][i])
            setpoints.append(Setpoint(time, position, velocity))
        return setpoints

    def table_to_setpoints(self, table_data):
        setpoints = []
        for i in range(0, table_data.columnCount()):
            time = float(table_data.item(0, i).text())
            position = math.radians(float(table_data.item(1, i).text()))
            velocity = math.radians(float(table_data.item(2, i).text()))
            setpoints.append(Setpoint(time, position, velocity))
        return setpoints

    def update_table(self, table, setpoints):
        rospy.logdebug("Updating table")

        table.setColumnCount(len(setpoints))
        for i in range(0, len(setpoints)):
            table.setItem(0, i, QTableWidgetItem(str(round(setpoints[i].time, self.TABLE_DIGITS))))
            table.setItem(1, i, QTableWidgetItem(str(round(math.degrees(setpoints[i].position), self.TABLE_DIGITS))))
            table.setItem(2, i, QTableWidgetItem(str(round(math.degrees(setpoints[i].velocity), self.TABLE_DIGITS))))

        table.resizeRowsToContents()
        table.resizeColumnsToContents()
        return table

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

    def publish_gait(self, ):
        trajectory = self.gait.to_joint_trajectory()
        rospy.loginfo("Publishing trajectory to topic '" + self.topic_name + "'")
        self.gait_publisher.publish(trajectory)

    def set_topic_name(self, topic_name):
        self.topic_name = topic_name
        self.gait_publisher = rospy.Publisher(topic_name,
                                              JointTrajectory, queue_size=10)

    def update_ui_elements(self, joint, table, plot, update_preview=True):
        plot.plot_item.blockSignals(True)
        table.blockSignals(True)

        plot.updateSetpoints(joint)

        self.update_table(table, joint.setpoints)

        plot.plot_item.blockSignals(False)
        table.blockSignals(False)

        if update_preview:
            self.publish_preview()

    def export_to_file(self):
        if self.gait_directory is None:
            self.gait_directory= str(QFileDialog.getExistingDirectory(None, "Select a directory to save gaits"))

        joint_trajectory = self.gait.to_joint_trajectory()
        output_file_directory = os.path.join(self.gait_directory, self.gait.name.replace(" ", "_"))
        output_file_path = os.path.join(output_file_directory, self.gait.version.replace(" ", "_") + ".gait")

        rospy.loginfo("Writing gait to " + output_file_path)

        try:
            os.makedirs(output_file_directory)
        except OSError:
            if not os.path.isdir(output_file_directory):
                raise

        output_file = open(output_file_path, 'w')
        output_file.write(str(joint_trajectory))

        output_file.close()


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

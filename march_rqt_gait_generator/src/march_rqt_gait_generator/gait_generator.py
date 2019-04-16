import math
import os
import sys

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QFrame
from python_qt_binding.QtWidgets import QTableWidgetItem


import rviz

from march_rqt_gait_generator.JointSettingPlot import JointSettingPlot
from march_rqt_gait_generator.model.Setpoint import Setpoint
from march_rqt_gait_generator.model.Joint import Joint
from march_rqt_gait_generator.model.Limits import Limits
from march_rqt_gait_generator.model.Gait import Gait

from urdf_parser_py import urdf

import pyqtgraph as pg

class GaitGeneratorPlugin(Plugin):
    TABLE_DIGITS = 4

    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)
        self.setObjectName('GaitGeneratorPlugin')

        self.robot = None
        self.load_urdf()
        self.gait = self.create_empty_gait()

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
        reader.readFile(config, os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'cfg.rviz'))
        self.frame.load(config)

        # Hide irrelevant Rviz details
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self._widget.RvizFrame.layout().addWidget(self.frame, 1, 0)

        self.create_joint_settings()

    def load_urdf(self):
        self.robot = urdf.Robot.from_parameter_server()

    def create_empty_gait(self):
        if self.robot is None:
            rospy.logerr("Cannot create gait without a loaded robot.")
        joint_list = []
        for i in range(0, len(self.robot.joints)):
            urdf_joint = self.robot.joints[i]

            if urdf_joint.limit is None:
                continue

            default_setpoints = [
                Setpoint(0, 0.6, 0.1),
                Setpoint(3.3, -0.1, 0.2),
                Setpoint(7.5, 1.3, 0.3),
                Setpoint(11.7, 0.5, 0.4),
                Setpoint(12, 0.3, 0.2)
            ]
            joint = Joint(urdf_joint.name,
                          Limits(urdf_joint.limit.upper, urdf_joint.limit.lower, urdf_joint.limit.velocity),
                          default_setpoints
                          )
            joint_list.append(joint)
        return Gait(joint_list)

    def create_joint_settings(self):
        for i in range(0, len(self.gait.joints)):
            self._widget.JointSettingContainer.layout().addWidget(self.create_joint_setting(self.gait.joints[i]), i % 3, i >= 3)

    def create_joint_setting(self, joint):
        joint_setting_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'joint_setting.ui')

        joint_setting = QFrame()
        loadUi(joint_setting_file, joint_setting)

        joint_setting_plot = JointSettingPlot(joint, self.gait.duration)

        # Connect a function to update the model and to update the table.
        joint_setting_plot.plot_item.sigPlotChanged.connect(
            lambda: [self.update_joint_setpoints(joint.name, self.plot_to_setpoints(joint_setting_plot)),
                     self.update_table(joint_setting.Table, self.gait.get_joint(joint.name).setpoints)
                    ])

        joint_setting.Plot.addItem(joint_setting_plot)

        # Populate table with data and resize
        joint_setting.Table = self.update_table(joint_setting.Table, joint.setpoints)

        joint_setting.Table.itemChanged.connect(
            lambda: [self.update_joint_setpoints(joint.name, self.table_to_setpoints(joint_setting_plot)),
                     self.update_plot(joint_setting_plot)
                     ])

        # Disable scrolling vertically
        joint_setting.Table.verticalScrollBar().setDisabled(True)

        return joint_setting

    def update_joint_setpoints(self, name, setpoints):
        self.gait.get_joint(name).set_setpoints(setpoints)

    def plot_to_setpoints(self, plot):
        plot_data = plot.plot_item.getData()
        setpoints = []
        for i in range(0, len(plot_data[0])):
            # TODO(Isha) Implement velocity here.
            setpoints.append(Setpoint(plot_data[0][i], plot_data[1][i], 0))
        return setpoints

    def table_to_setpoints(self, table):
        table_data = table
        setpoints = []
        rospy.logwarn(table_data)
        # for i in range(0, len(plot_data[0])):
        #     TODO(Isha) Implement velocity here.
            # setpoints.append(Setpoint(plot_data[0][i], plot_data[1][i], 0))
        return setpoints

    def update_plot(self, plot):
        rospy.loginfo("Updating plot")
        plot.plot_item.setData([0.1,0.2,0.3,0.4,0.5], [0.1,0.2,0.3,0.4,0.5])


    def update_table(self, table, setpoints):
        rospy.loginfo("Updating table")

        table.setColumnCount(len(setpoints))
        for i in range(0, len(setpoints)):
            table.setItem(0, i, QTableWidgetItem(str(round(setpoints[i].time, self.TABLE_DIGITS))))
            table.setItem(1, i, QTableWidgetItem(str(round(setpoints[i].position, self.TABLE_DIGITS))))
            table.setItem(2, i, QTableWidgetItem(str(round(setpoints[i].velocity, self.TABLE_DIGITS))))

        table.resizeRowsToContents()
        table.resizeColumnsToContents()
        return table

# def trigger_configuration(self):
# Comment in to signal that the plugin has a way to configure
# This will enable a setting button (gear icon) in each dock widget title bar
# Usually used to open a modal configuration dialog

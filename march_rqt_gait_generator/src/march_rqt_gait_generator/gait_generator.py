import os
import sys

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QRadioButton
from python_qt_binding.QtWidgets import QCheckBox

import rviz


class GaitGeneratorPlugin(Plugin):

    def __init__(self, context):
        super(GaitGeneratorPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GaitGeneratorPlugin')

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

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'gait_generator.ui')

        # Extend the widget with all attributes and children from UI file

        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names

        self._widget.setObjectName('Gait Generator')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self.frame = rviz.VisualizationFrame()
        # self.frame.setSplashPath( "" )
        self.frame.initialize()


        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, os.path.join(rospkg.RosPack().get_path('march_rqt_gait_generator'), 'resource', 'cfg.rviz'))
        self.frame.load(config)

        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self._widget.RvizFrame.layout().addWidget(self.frame)



    """Return all widgets found in the requested layout."""
    def get_layout_widgets(self, layout):
        return (layout.itemAt(i).widget() for i in range(layout.count()))

# def trigger_configuration(self):
# Comment in to signal that the plugin has a way to configure
# This will enable a setting button (gear icon) in each dock widget title bar
# Usually used to open a modal configuration dialog

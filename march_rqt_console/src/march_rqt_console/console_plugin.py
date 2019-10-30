import os

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget


class ConsolePlugin(Plugin):

    def __init__(self, context):
        super(ConsolePlugin, self).__init__(context)
        self.setObjectName('Console')

        self._widget = QWidget()
        self.init_ui(context)

    def init_ui(self, context):
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_console'), 'resource', 'console.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('ConsoleUi')
        context.add_widget(self._widget)

import os

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from march_rqt_console.message import Message
from march_rqt_console.message_model import MessageModel


class ConsolePlugin(Plugin):

    def __init__(self, context):
        super(ConsolePlugin, self).__init__(context)

        self._widget = QWidget()
        self.init_ui(context)

        self._model = MessageModel()

        self._widget.table_view.setModel(self._model)
        self._widget.table_view.verticalScrollBar().rangeChanged.connect(self.change_scroll)
        self._widget.input_field.returnPressed.connect(self.insert_message)

    def init_ui(self, context):
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_console'), 'resource', 'console.ui')
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

    def insert_message(self):
        msg = self._widget.input_field.text()
        if msg:
            self._model.insert_row(Message(msg))
            self._widget.input_field.clear()

    def change_scroll(self, min, max):
        self._widget.table_view.verticalScrollBar().setSliderPosition(max)

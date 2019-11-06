import os

import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from march_rqt_note_taker.entry import Entry
from march_rqt_note_taker.entry_model import EntryModel


class NotesPlugin(Plugin):

    def __init__(self, context):
        super(NotesPlugin, self).__init__(context)

        self._widget = QWidget()
        self.init_ui(context)

        self._model = EntryModel()

        self._widget.table_view.setModel(self._model)
        self._widget.table_view.verticalScrollBar().rangeChanged.connect(self.change_scroll)
        self._widget.input_field.returnPressed.connect(self.insert_entry)

    def init_ui(self, context):
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_note_taker'), 'resource', 'plugin.ui')
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

    def insert_entry(self):
        entry = self._widget.input_field.text()
        if entry:
            self._model.insert_row(Entry(entry))
            self._widget.input_field.clear()

    def change_scroll(self, min, max):
        self._widget.table_view.verticalScrollBar().setSliderPosition(max)

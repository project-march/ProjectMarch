import os

from rosgraph_msgs.msg import Log
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget

from .entry import Entry
from .entry_model import EntryModel


class NotesPlugin(Plugin):

    def __init__(self, context):
        super(NotesPlugin, self).__init__(context)

        self._widget = QWidget()
        self.init_ui(context)

        self._model = EntryModel()
        self._model.rowsInserted.connect(self.update_status)
        self._model.rowsRemoved.connect(self.update_status)

        self._widget.table_view.setModel(self._model)
        self._widget.table_view.verticalScrollBar().rangeChanged.connect(self.change_scroll)

        self._widget.input_field.returnPressed.connect(self.insert_entry)

        self._widget.take_button.clicked.connect(self.start_take)

        self._subscriber = rospy.Subscriber('/rosout_agg',
                                            Log,
                                            lambda l: self._model.insert_row(Entry.from_ros_msg(l)))

    def init_ui(self, context):
        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_note_taker'), 'resource', 'note_taker.ui')
        loadUi(ui_file, self._widget)

        context.add_widget(self._widget)

    def insert_entry(self):
        entry = self._widget.input_field.text().strip()
        if entry:
            self._model.insert_row(Entry(entry))
        self._widget.input_field.clear()

    def update_status(self):
        self._widget.messages_label.setText('Displaying {} messages'.format(self._model.rowCount()))

    def start_take(self):
        take = self._widget.camera_spin_box.value()
        self._model.insert_row(Entry('Started camera take {}'.format(take)))

    def change_scroll(self, scroll_min, scroll_max):
        self._widget.table_view.verticalScrollBar().setSliderPosition(scroll_max)

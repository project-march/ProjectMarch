import os

from rosgraph_msgs.msg import Log
import rospkg
import rospy

from qt_gui.plugin import Plugin

from .entry_model import EntryModel
from .filter import Filter
from .notes_widget import NotesWidget


class NotesPlugin(Plugin):

    def __init__(self, context):
        super(NotesPlugin, self).__init__(context)

        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_note_taker'), 'resource', 'note_taker.ui')

        self._model = EntryModel()
        self._widget = NotesWidget(self._model, ui_file)
        context.add_widget(self._widget)

        self._filter = Filter()
        self._filter.add_filter(lambda l: l.level >= Log.ERROR)
        self._filter.add_filter_info_level(lambda l: l.msg == 'March is fully operational')
        self._subscriber = rospy.Subscriber('/rosout_agg',
                                            Log,
                                            self._rosout_cb)

    def shutdown_plugin(self):
        self._subscriber.unregister()

    def _rosout_cb(self, log_msg):
        if self._filter.apply(log_msg):
            self._model.insert_log_msg(log_msg)

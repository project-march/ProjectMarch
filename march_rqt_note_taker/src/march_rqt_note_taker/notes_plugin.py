import os

from qt_gui.plugin import Plugin
from rosgraph_msgs.msg import Log
import rospkg
import rospy

from .entry_model import EntryModel
from .filter_map import FilterMap
from .notes_widget import NotesWidget


class NotesPlugin(Plugin):

    def __init__(self, context):
        super(NotesPlugin, self).__init__(context)

        ui_file = os.path.join(rospkg.RosPack().get_path('march_rqt_note_taker'), 'resource', 'note_taker.ui')

        self._model = EntryModel()
        self._widget = NotesWidget(self._model, ui_file)
        context.add_widget(self._widget)

        self._filter_map = FilterMap()
        self._filter_map.add_filter(lambda l: l.level >= Log.ERROR)
        self._filter_map.add_filter_info_level(lambda l: l.msg == 'March is fully operational')
        self._subscriber = rospy.Subscriber('/rosout_agg',
                                            Log,
                                            self._rosout_cb)

    def shutdown_plugin(self):
        self._subscriber.unregister()

    def _rosout_cb(self, log_msg):
        mapped_msg = self._filter_map(log_msg)
        if mapped_msg:
            self._model.insert_log_msg(mapped_msg)

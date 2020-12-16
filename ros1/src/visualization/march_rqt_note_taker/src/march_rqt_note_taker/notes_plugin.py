import ast
import os

from qt_gui.plugin import Plugin
from rosgraph_msgs.msg import Log
import rospkg
import rospy
from std_srvs.srv import Trigger

from march_shared_msgs.msg import CurrentState

from .entry import Entry
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

        self._subscribers = []
        self._subscribers.append(rospy.Subscriber('/rosout_agg',
                                                  Log,
                                                  self._rosout_cb))
        self._subscribers.append(rospy.Subscriber('/march/gait_selection/current_state',
                                                  CurrentState,
                                                  self._current_state_cb))

        self._get_gait_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map',
                                                        Trigger)

    def shutdown_plugin(self):
        for subscriber in self._subscribers:
            subscriber.unregister()

    def _rosout_cb(self, log_msg):
        mapped_msg = self._filter_map(log_msg)
        if mapped_msg:
            self._model.insert_log_msg(mapped_msg)

    def _current_state_cb(self, current_state):
        """
        Inserts an entry, which logs the current gait version used.

        :param current_state: Current state being executed
        """
        if current_state.state_type == CurrentState.IDLE:
            message = 'March is idle in {state}'.format(state=current_state.state)
            self._model.insert_row(Entry(message))
        elif current_state.state_type == CurrentState.GAIT:
            try:
                gait_version_map = ast.literal_eval(self._get_gait_version_map().message)

                message = 'Starting gait {0}: {1}'.format(current_state.state,
                                                          str(gait_version_map[current_state.state]))
                self._model.insert_row(Entry(message))
            except rospy.ServiceException as e:
                rospy.logwarn('Failed to contact gait selection node for gait versions: {0}'.format(e))
            except KeyError:
                pass
            except ValueError as e:
                rospy.logerr('Failed to parse gait version map: {0}'.format(e))

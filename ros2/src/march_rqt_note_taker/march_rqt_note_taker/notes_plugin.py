import ast
import os

from ament_index_python import get_package_share_directory
from qt_gui.plugin import Plugin

import rclpy
from rcl_interfaces.msg import Log
from rclpy.exceptions import InvalidServiceNameException
from rclpy.node import Node
from std_srvs.srv import Trigger
from rqt_gui.main import Main
import sys
from march_shared_msgs.msg import CurrentState

from rclpy.task import Future
from .entry import Entry
from .entry_model import EntryModel
from .filter_map import FilterMap
from .notes_widget import NotesWidget


def main(args=None):
    """
    The main function used to start up the rqt input device.
    """
    rclpy.init(args=args)

    try:
        plugin = 'march_rqt_note_taker'
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(standalone=plugin))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


class NotesPlugin(Plugin):

    def __init__(self, context):
        super(NotesPlugin, self).__init__(context)

        ui_file = os.path.join(get_package_share_directory('march_rqt_note_taker'), 'note_taker.ui')

        self._node: Node = context.node

        self._model = EntryModel()
        self._widget = NotesWidget(self._model, ui_file, self._node)
        context.add_widget(self._widget)

        self._filter_map = FilterMap()
        self._filter_map.add_filter(lambda l: l.level >= Log.ERROR)
        self._filter_map.add_filter_info_level(lambda l: l.msg == 'March is fully operational')

        self._node.create_subscription(Log, '/rosout_agg', self._rosout_cb, qos_profile=10)
        self._node.create_subscription(CurrentState, '/march/gait_selection/current_state',
                                       self._current_state_cb, qos_profile=10)

        self._get_gait_version_map_client = self._node.create_client(Trigger, '/march/gait_selection/get_version_map')

    def shutdown_plugin(self):
        for subscriber in self._node.subscriptions:
            subscriber.destroy()

    def _rosout_cb(self, log_msg: Log):
        mapped_msg = self._filter_map(log_msg)
        if mapped_msg:
            self._model.insert_log_msg(mapped_msg)

    def _current_state_cb(self, current_state):
        """
        Inserts an entry, which logs the current gait version used.

        :param current_state: Current state being executed
        """
        if current_state.state_type == CurrentState.IDLE:
            message = f'March is idle in {current_state.state}'
            self._model.insert_row(Entry(message))
        elif current_state.state_type == CurrentState.GAIT:
            try:
                future = self._get_gait_version_map_client.call_async(Trigger.Request())
                future.add_done_callback(lambda future_done: self._get_version_map_callback(future_done, current_state))
            except InvalidServiceNameException as error:
                self._node.get_logger().warn(f'Failed to contact gait selection node for gait versions: {error}')
            except KeyError:
                pass
            except ValueError as error:
                self._node.get_logger().error(f'Failed to parse gait version map: {error}')

    def _get_version_map_callback(self, future_done: Future, current_state):
        self._node.get_logger().info('done with future')
        result = future_done.result()

        if result.success:
            gait_version_map = ast.literal_eval(result.message)
            message = 'Starting gait {0}: {1}'.format(current_state.state,
                                                      str(gait_version_map[current_state.state]))
            self._model.insert_row(Entry(message))

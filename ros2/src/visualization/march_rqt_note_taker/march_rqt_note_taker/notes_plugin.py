import ast
import os
import sys


from ament_index_python import get_package_share_directory

from march_shared_msgs.msg import CurrentState

from qt_gui.plugin import Plugin

from rcl_interfaces.msg import Log
from rcl_interfaces.srv import GetParameters

import rclpy
from rclpy.exceptions import InvalidServiceNameException
from rclpy.node import Node
from rclpy.task import Future

from rqt_gui.main import Main

from std_srvs.srv import Trigger

from .entry import Entry
from .entry_model import EntryModel
from .filter_map import FilterMap
from .notes_widget import NotesWidget


def main(args=None):
    """The main function used to start up the rqt note taker."""
    rclpy.init(args=args)

    try:
        plugin = "march_rqt_note_taker"
        main_plugin = Main(filename=plugin)
        sys.exit(main_plugin.main(standalone=plugin))

    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


class NotesPlugin(Plugin):
    def __init__(self, context):
        """Initialize the NotesPLugin."""
        super(NotesPlugin, self).__init__(context)

        ui_file = os.path.join(
            get_package_share_directory("march_rqt_note_taker"), "note_taker.ui"
        )

        self._node: Node = context.node

        self._model = EntryModel()
        self._widget = NotesWidget(self._model, ui_file, self._node)
        context.add_widget(self._widget)

        self._use_current_time = self._should_use_current_time()

        # Log a message when it is an error,
        # or when the content is 'March is fully operational'
        self._filter_map = FilterMap()
        self._filter_map.add_filter_on_minimal_level(Log.ERROR)
        self._filter_map.add_filter_on_level(
            level=Log.INFO, msg_filter=lambda l: l.msg == "March is fully operational"
        )

        self._node.create_subscription(
            Log, "/rosout_agg", self._rosout_cb, qos_profile=10
        )
        self._node.create_subscription(
            CurrentState,
            "/march/gait_selection/current_state",
            self._current_state_cb,
            qos_profile=10,
        )

        self._get_gait_version_map_client = self._node.create_client(
            Trigger, "/march/gait_selection/get_version_map"
        )

    def _should_use_current_time(self) -> bool:
        """Determine whether the rqt_note_taker should use the current time
        when creating a new entry or the timestamp of the log msg.

        This is determined based on whether the use_sim_time parameter is True.
        When this parameter is true, the log msg will not have
        an useful timestamp, meaning the current time should be used.

        If the node is launched via the launch file, then the parameter can
        simply be retrieved from the node's parameter server.
        If the node is launch via rqt_gui, it will not have the parameter
        correctly set in its own parameter server, therefore the parameter
        is retrieved from the march_monitor node.

        :return Returns a boolean, indicating whether the current time should
        be used when creating a new entry.
        """
        client = self._node.create_client(
            GetParameters, "/march_monitor/get_parameters"
        )
        if client.service_is_ready():
            future = client.call_async(GetParameters.Request(names=["use_sim_time"]))
            rclpy.spin_until_future_complete(self._node, future)
            client.destroy()
            return future.result().values[0].bool_value
        else:
            return (
                self._node.get_parameter("use_sim_time")
                .get_parameter_value()
                .bool_value
            )

    def shutdown_plugin(self):
        """Close the plugin.

        Destroy all subscribers of the node.
        """
        for subscriber in self._node.subscriptions:
            subscriber.destroy()

    def _rosout_cb(self, log_msg: Log):
        """Callback for the /rosout_agg topic.

        If the message is accepted by the filter map, the message is added as a
        new entry to the model.

        :param log_msg Log message in the topic.
        """
        mapped_msg = self._filter_map(log_msg)
        if mapped_msg:
            self._model.insert_log_msg(mapped_msg, self._use_current_time)

    def _current_state_cb(self, current_state):
        """Callback for when the current state changes.

        After the current state is changed this callback does either
        1) Log that march is in idle state, or
        2) Log the gait that is selected, by getting the version_map and
        creating a new entry.

        :param current_state: Current state being executed
        """
        if current_state.state_type == CurrentState.IDLE:
            message = f"March is idle in {current_state.state}"
            self._model.insert_row(Entry(message))
        elif current_state.state_type == CurrentState.GAIT:
            try:
                future = self._get_gait_version_map_client.call_async(Trigger.Request())
                future.add_done_callback(
                    lambda future_done: self._get_version_map_callback(
                        future_done, current_state
                    )
                )
            except InvalidServiceNameException as error:
                self._node.get_logger().warn(
                    f"Failed to contact gait selection node "
                    f"for gait versions: {error}"
                )

    def _get_version_map_callback(self, future_done: Future, current_state):
        """Callback for when the version map is retrieved.

        Parses the result of the future and inserts a new entry.

        :param future_done Future that holds the result
        """
        result = future_done.result()

        if result.success:
            try:
                gait_version_map = ast.literal_eval(result.message)
                message = (
                    f"Starting gait {current_state.state}: "
                    f"{gait_version_map[current_state.state]}"
                )
                self._model.insert_row(Entry(message))
            except KeyError:
                pass
            except ValueError as error:
                self._node.get_logger().error(
                    f"Failed to parse gait version map: {error}"
                )

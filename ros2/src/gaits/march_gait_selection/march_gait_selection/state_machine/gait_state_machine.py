"""Author: Marten Haitjema."""

from typing import Optional, Dict
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.timer import Timer
from rclpy.node import Node

from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_walk import DynamicGaitWalk
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.state_machine_input import StateMachineInput
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler

from march_utility.gait.edge_position import UnknownEdgePosition, EdgePosition
from march_utility.utilities.shutdown import shutdown_system
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

from march_shared_msgs.msg import CurrentState, CurrentGait, Error
from march_shared_msgs.srv import PossibleGaits
from std_msgs.msg import Header


class GaitStateMachine:
    """Class that keeps track of the current gait state and schedules new gaits, if possible.

    Args:
        node (Node): the gait_node
        trajectory_scheduler (TrajectoryScheduler): class to publish trajectory commands
        gaits (dict): dictionary containing the name and class instance of all loaded gaits
        positions (dict): dictionary containing the name and the EdgePosition of all loaded positions

    Attributes:
        _node (Node): the gait_node
        _gaits (dict): dictionary containing the name and class instance of all loaded gaits
        _positions (dict): dictionary containing the name and the EdgePosition of all loaded positions
        _logger (rclpy.Logger): used to log to the terminal
        _input (StateMachineInput): class that handles the input to the state machine
        _timer_period (double): update period that the state machine runs on
        _last_end_position (EdgePosition): final position of all joints of the last executed gait
        _current_gait (Gait): instance of the gait class that is currently being executed
        _shutdown_requested (bool): whether the state machine should shut down (in case of an error)
        _should_stop (bool): whether the current gait should be stopped (with a close gait)
        _is_stopping (bool): whether the current gait is already stopping
        _accepted_gait (bool): whether a gait has been accepted
        _executing_gait (bool): whether the accepted gait has started executing
        _previous_gait (Gait): instance of the gait class that has previously been executed
        _current_state_pub (rclpy.Publisher): publisher to publish on the current state topic
        _current_gait_pub (rclpy.Publisher): publisher to publish on the current gait topic
        _error_sub (rclpy.Subscriber): subscriber of /march/error topic
        _get_possible_gaits_client (rclpy.Service): service to send possible gaits to input device
        _update_timer (rclpy.Timer): timer on which the update method is called
    """

    _update_timer: Timer
    _should_stop: bool
    _shutdown_requested: bool

    def __init__(
        self, node: Node, trajectory_scheduler: TrajectoryScheduler, gaits: dict, positions: Dict[EdgePosition, str]
    ):
        self._node = node
        self._trajectory_scheduler = trajectory_scheduler
        self._gaits = gaits
        self._positions = positions
        self._previous_gait = None

        self._logger = node.get_logger().get_child(__class__.__name__)
        self._input = StateMachineInput(node)
        self._timer_period = self._node.get_parameter("timer_period").get_parameter_value().double_value
        self._last_end_position = UnknownEdgePosition()
        self._reset_attributes()

        self._current_state_pub = self._node.create_publisher(
            msg_type=CurrentState,
            topic="/march/gait_selection/current_state",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._current_gait_pub = self._node.create_publisher(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._error_sub = self._node.create_subscription(
            msg_type=Error,
            topic="/march/error",
            callback=self._error_cb,
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._get_possible_gaits_client = self._node.create_service(
            srv_type=PossibleGaits,
            srv_name="/march/gait_selection/get_possible_gaits",
            callback=self._possible_gaits_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    def _reset_attributes(self) -> None:
        """Resets attributes."""
        self._current_gait = None
        self._shutdown_requested = False
        self._should_stop = False
        self._is_stopping = False
        self._accepted_gait = False
        self._executing_gait = False

    def run(self) -> None:
        """Runs the state machine until shutdown is requested."""
        self._update_timer = self._node.create_timer(timer_period_sec=self._timer_period, callback=self.update)

    def update(self) -> None:
        """Updates the current state each timer period, after the state machine is started."""
        if self._shutdown_requested:
            self._update_timer.cancel()
            return

        if self._input.unknown_requested():
            self._handle_unknown_requested()
        elif self._accepted_gait:
            self._process_gait_state()
        else:
            self._process_idle_state()

    def _process_idle_state(self) -> None:
        """If the current state is idle, this function processes input for what to do next."""
        if self._input.gait_requested():
            self._current_gait = self._gaits.get(self._input.gait_name())
            self._process_gait_request()
        elif self._previous_gait is not None and self._is_dynamic_stop_requested():
            self._current_gait = self._gaits.get("dynamic_close")
            self._process_gait_request()

    def _process_gait_request(self) -> None:
        """Accepts the requested gait if the starting positions equals the last end position."""
        self._logger.info(f"Requested gait `{self._current_gait.name}`")
        if self._current_gait.starting_position == self._last_end_position:
            self._input.gait_accepted()
            self._publish_gait_state()
            self._accepted_gait = True
            self._logger.info(f"Accepted gait `{self._current_gait.name}`")
        else:
            self._input.gait_rejected()
            self._logger.info(
                f"Cannot execute gait `{self._current_gait.name}` from idle state `{self._last_end_position}`"
            )

    def _is_dynamic_stop_requested(self) -> bool:
        """Returns true if a stop is requested and the previous gait was a step or step and hold."""
        return (
            self._is_stop_requested()
            and not self._is_stopping
            and self._previous_gait.requires_dynamic_stop
            and not isinstance(self._last_end_position, UnknownEdgePosition)
        )

    def _process_gait_state(self) -> None:
        """Processes the current state when there is a gait happening.

        Schedules the next subgait if there is no trajectory happening or
        finishes the gait if it is done.
        """
        self._handle_stop_input()
        if self._trajectory_scheduler.failed():
            self._process_end_of_gait()
            self._transition_to_unknown()
            return

        now = self._node.get_clock().now()
        if not self._executing_gait:
            try:
                gait_update = self._current_gait.start(now, self._node.first_subgait_delay)
            except (AttributeError, ValueError) as e:
                self._logger.error(f"Gait cannot be started due to an error: {e}. Transitioning to unknown state.")
                self._input.gait_finished()
                self._handle_unknown_requested()
                return

            self._executing_gait = True

            if gait_update == GaitUpdate.empty():
                self._input.gait_finished()
                self._reset_attributes()
                self._logger.warn(
                    f"Starting the gait returned no trajectory, going back to previous "
                    f"idle state {self._last_end_position}"
                )
                return
        else:
            try:
                gait_update = self._current_gait.update(now, self._node.early_schedule_duration)
            except (AttributeError, ValueError) as e:
                self._logger.error(f"Calling update of gait failed: {e}. Transitioning to unknown state.")
                self._input.gait_finished()
                self._handle_unknown_requested()
                return

        self._process_gait_update(gait_update)

    def _handle_stop_input(self) -> None:
        """Handles stop input from the input device.

        This input is passed on to the current gait to execute the request.
        """
        if self._is_stop_requested() and not self._is_stopping:
            self._should_stop = False
            self._is_stopping = True
            if self._current_gait.stop():
                self._logger.info(f"Gait {self._current_gait.name} responded to stop")
                self._input.stop_accepted()
            else:
                self._logger.info(f"Gait {self._current_gait.name} does not respond to stop")
                self._input.stop_rejected()

    def _process_gait_update(self, gait_update: GaitUpdate) -> None:
        """Process an incoming GaitUpdate.

        Will call gait callbacks for each new subgait, schedule new trajectories and process finishing of gait.

        Args:
            gait_update (GaitUpdate): GaitUpdate returned by the gait that is currently executing
        """
        if gait_update.is_new_subgait:
            self._publish_current_gait()
        if gait_update.new_trajectory_command is not None:
            self._trajectory_scheduler.schedule(gait_update.new_trajectory_command)
        if gait_update.is_finished:
            self._previous_gait = self._current_gait
            self._last_end_position = self._current_gait.final_position
            self._publish_idle_state()
            self._process_end_of_gait()
            self._logger.info(f"Finished gait `{self._previous_gait.name}`")

    def _transition_to_unknown(self) -> None:
        """When the unknown button is pressed, this function resets the state machine to unknown state."""
        if self._current_gait is not None:
            self._trajectory_scheduler.send_position_hold()
            self._trajectory_scheduler.cancel_active_goals()

        if isinstance(self._current_gait, DynamicGaitWalk):
            self._current_gait.set_state_to_unknown()

        self._last_end_position = UnknownEdgePosition()
        self._reset_attributes()
        self._logger.info("Transitioned to unknown")

    def _is_stop_requested(self) -> bool:
        """Returns true if either the input device requested a stop or some other external source requested a stop.

        Returns:
            bool: True if stop is requested, else False
        """
        return self._input.stop_requested() or self._should_stop

    def _error_cb(self, msg: Error) -> None:
        """Standard callback for state machine errors.

        Completely stops updating if the error is fatal. If non-fatal, it stops the current gait.

        Args:
            msg (Error): message containing the error type (fatal or non-fatal)
        """
        if msg.type == Error.NON_FATAL:
            self.stop_gait()
        elif msg.type == Error.FATAL:
            self.request_shutdown("A fatal error was posted to /march/error")

    def stop_gait(self) -> None:
        """Requests a stop from the current executing gait, but keeps the state machine running."""
        if self._accepted_gait and not self._is_stopping:
            self._should_stop = True

    def request_shutdown(self, msg: Optional[str] = None) -> None:
        """Requests shutdown, which will terminate the state machine as soon as possible.

        Args:
            msg (:obj: str, optional): message to log with shutdown request, if not specified will be
                "Shutdown requested"
        """
        base_msg = "Shutdown requested"
        if msg is not None:
            base_msg += ": " + msg
        self._logger.fatal(base_msg)
        self._shutdown_requested = True
        shutdown_system()

    def _handle_unknown_requested(self) -> None:
        """Accept gait, set state to unknown and reset trajectory_scheduler."""
        self._input.gait_accepted()
        self._transition_to_unknown()
        self._input.gait_finished()
        self._publish_idle_state()
        self._trajectory_scheduler.reset()

    def _process_end_of_gait(self) -> None:
        """Resets trajectory_scheduler, end the current gait and calls gait_finished on input device."""
        self._trajectory_scheduler.reset()
        self._current_gait.end()
        self._input.gait_finished()
        self._reset_attributes()

    def _publish_gait_state(self) -> None:
        """Publish the name of the gait that is currently executing."""
        self._current_state_pub.publish(
            CurrentState(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                state=self._current_gait.name,
                state_type=CurrentState.GAIT,
            )
        )

    def _publish_idle_state(self) -> None:
        """Publish the name of the position of the current idle state and publish the joint_angles."""
        if self._last_end_position in self._positions:
            state = self._positions[self._last_end_position]
        else:
            state = f"unnamed: {self._last_end_position}"

        self._current_state_pub.publish(
            CurrentState(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                state=state,
                state_type=CurrentState.IDLE,
            )
        )

    def _publish_current_gait(self) -> None:
        """Standard callback when gait changes, publishes the current gait."""
        self._current_gait_pub.publish(
            CurrentGait(
                header=Header(stamp=self._node.get_clock().now().to_msg()),
                gait=self._current_gait.name,
                subgait=self._current_gait.subgait_name,
                version=self._current_gait.version,
                duration=self._current_gait.duration.to_msg(),
                gait_type=self._current_gait.gait_type,
            )
        )

    def _possible_gaits_cb(self, request, response: PossibleGaits) -> PossibleGaits:
        """Standard callback for the get possible gaits service.

        Args:
            request: Necessary for service
            response (PossibleGaits): PossibleGaits message to return at service request
        """
        possible_gaits = []
        if not self._accepted_gait:
            for gait in self._gaits.values():
                if self._last_end_position == gait.starting_position:
                    possible_gaits.append(gait.name)

        response.gaits = possible_gaits
        return response

    def update_parameters(self, gait_name: str) -> None:
        """Update dynamic reconfigure parameters in gait classes.

        Args:
            gait_name (str): Name of the gait of which the parameters should be updated.
        """
        if gait_name in self._gaits:
            self._gaits[gait_name].update_parameters()

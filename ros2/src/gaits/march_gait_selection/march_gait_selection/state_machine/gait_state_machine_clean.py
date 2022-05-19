"""Author: Marten Haitjema."""

from typing import Optional

from rclpy.callback_groups import ReentrantCallbackGroup

from march_gait_selection.gait_selection_clean import GaitSelectionClean
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.state_machine_input import StateMachineInput
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler
from march_shared_msgs.srv import PossibleGaits

from march_utility.gait.edge_position import UnknownEdgePosition
from march_utility.utilities.logger import Logger
from march_utility.utilities.shutdown import shutdown_system
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

from sensor_msgs.msg import JointState
from march_shared_msgs.msg import CurrentState, CurrentGait, Error
from std_msgs.msg import Header


class GaitStateMachineClean:
    """Clean version of the state machine that can only be used with limited gaits."""

    def __init__(self, gait_selection: GaitSelectionClean, trajectory_scheduler: TrajectoryScheduler):
        self._gait_selection = gait_selection
        self._trajectory_scheduler = trajectory_scheduler
        self._logger = Logger(self._gait_selection, __class__.__name__)
        self._input = StateMachineInput(gait_selection)
        self._timer_period = self._gait_selection.get_parameter("timer_period").get_parameter_value().double_value

        self._shutdown_requested = False
        self._should_stop = False
        self._is_stopping = False

        self._current_gait = None
        self._last_idle_position = UnknownEdgePosition()
        self._executing_gait = False
        self._has_gait_started = False

        self.current_state_pub = self._gait_selection.create_publisher(
            msg_type=CurrentState,
            topic="/march/gait_selection/current_state",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self.current_gait_pub = self._gait_selection.create_publisher(
            msg_type=CurrentGait,
            topic="/march/gait_selection/current_gait",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self.final_position_pub = self._gait_selection.create_publisher(
            msg_type=JointState,
            topic="/march/gait_selection/final_position",
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self.error_sub = self._gait_selection.create_subscription(
            msg_type=Error,
            topic="/march/error",
            callback=self._error_cb,
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._get_possible_gaits_client = self._gait_selection.create_service(
            srv_type=PossibleGaits,
            srv_name="/march/gait_selection/get_possible_gaits",
            callback=self._possible_gaits_cb,
            callback_group=ReentrantCallbackGroup(),
        )

    def run(self) -> None:
        """Runs the state machine until shutdown is requested."""
        self.update_timer = self._gait_selection.create_timer(timer_period_sec=self._timer_period, callback=self.update)

    def update(self) -> None:
        """Updates the current state each timer period, after the state machine is started."""
        if not self._shutdown_requested:
            if self._input.unknown_requested():
                self._input.gait_accepted()
                self._transition_to_unknown()
                self._input.gait_finished()
                self._publish_idle_state()
                self._current_gait = None
                self._trajectory_scheduler.reset()
            elif not self._executing_gait:
                self._process_idle_state()
            else:
                self._process_gait_state()
        else:
            self.update_timer.cancel()

    def _process_idle_state(self) -> None:
        """If the current state is idle, this function processes input for what to do next."""
        if self._input.gait_requested():
            self._current_gait = self._gait_selection.gaits.get(self._input.gait_name())
            self._logger.info(f"Requested gait `{self._current_gait.name}`")
            if self._current_gait.starting_position == self._last_idle_position:
                self._should_stop = False
                self._input.gait_accepted()
                self._publish_gait_state()
                self._executing_gait = True
                self._logger.info(f"Accepted gait `{self._current_gait.name}`")
            else:
                self._input.gait_rejected()
                self._logger.info(
                    f"Cannot execute gait `{self._current_gait.name}` from idle state `{self._current_state}`"
                )

    def _process_gait_state(self) -> None:
        """Processes the current state when there is a gait happening.

        Schedules the next subgait if there is no trajectory happening or
        finishes the gait if it is done.
        """
        self._handle_stop_input()
        if self._trajectory_scheduler.failed():
            self._trajectory_scheduler.reset()
            self._current_gait.end()
            self._current_gait = None
            self._transition_to_unknown()
            self._input.gait_finished()
            return

        now = self._gait_selection.get_clock().now()
        if not self._has_gait_started:
            gait_update = self._current_gait.start(now, self._gait_selection._first_subgait_delay)
            self._has_gait_started = True

            if gait_update == GaitUpdate.empty():
                self._input.gait_finished()
                self._executing_gait = False
                self._has_gait_started = False
                self._current_gait = None
                self._logger.info(
                    f"Starting the gait returned no trajectory, going back to previous "
                    f"idle state {self._last_idle_position}"
                )
                return
        else:
            gait_update = self._current_gait.update(now, self._gait_selection._early_schedule_duration)

        self._process_gait_update(gait_update)

    def _handle_stop_input(self) -> None:
        """Handles stop input from the input device.

        This input is passed on to the current gait to execute the request.
        """
        if self._is_stop_requested() and not self._is_stopping:
            if self._previous_gait.name == "dynamic_step" and not isinstance(self._current_state, UnknownEdgePosition):
                self._current_state = "dynamic_close"
            else:
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
            self._last_idle_position = self._current_gait.final_position
            self._previous_gait = self._current_gait
            self.final_position_pub.publish(JointState(position=self._last_idle_position.values))
            self._publish_idle_state()
            self._current_gait.end()
            self._input.gait_finished()
            self._trajectory_scheduler.reset()
            self._executing_gait = False
            self._has_gait_started = False
            self._logger.info(f"Finished gait `{self._current_gait.name}`")
            self._current_gait = None
            self._is_stopping = False

    def _transition_to_unknown(self) -> None:
        """When the unknown button is pressed, this function resets the state machine to unknown state."""
        if self._current_gait is not None:
            self._trajectory_scheduler.send_position_hold()
            self._trajectory_scheduler.cancel_active_goals()
        self._current_state = UnknownEdgePosition()
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
        if self._executing_gait and not self._is_stopping:
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

    def _publish_gait_state(self) -> None:
        self.current_state_pub.publish(
            CurrentState(
                header=Header(stamp=self._gait_selection.get_clock().now().to_msg()),
                state=self._current_gait.name,
                state_type=CurrentState.GAIT,
            )
        )

    def _publish_idle_state(self) -> None:
        if self._last_idle_position in self._gait_selection.positions:
            state = self._gait_selection.positions[self._last_idle_position]
        else:
            state = f"unnamed: {self._last_idle_position}"

        self.current_state_pub.publish(
            CurrentState(
                header=Header(stamp=self._gait_selection.get_clock().now().to_msg()),
                state=state,
                state_type=CurrentState.IDLE,
            )
        )

    def _publish_current_gait(self) -> None:
        """Standard callback when gait changes, publishes the current gait."""
        self.current_gait_pub.publish(
            CurrentGait(
                header=Header(stamp=self._gait_selection.get_clock().now().to_msg()),
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
        if not self._executing_gait:
            for gait in self._gait_selection.gaits.values():
                if self._last_idle_position == gait.starting_position:
                    possible_gaits.append(gait.name)

        response.gaits = possible_gaits
        return response

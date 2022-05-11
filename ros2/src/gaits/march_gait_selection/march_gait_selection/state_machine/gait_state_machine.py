"""Author: ???."""

from typing import Optional, Union, List
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState

from march_gait_selection.state_machine.state_machine_input import StateMachineInput
from march_shared_msgs.msg import CurrentState, CurrentGait, Error
from march_shared_msgs.srv import PossibleGaits
from march_utility.gait.edge_position import (
    EdgePosition,
    UnknownEdgePosition,
    DynamicEdgePosition,
)
from march_utility.gait.gait_graph import GaitGraph
from march_utility.utilities.duration import Duration
from march_utility.utilities.shutdown import shutdown_system
from march_utility.utilities.side import Side
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Header
from .gait_update import GaitUpdate
from .trajectory_scheduler import TrajectoryScheduler
from ..gait_selection import GaitSelection

State = Union[EdgePosition, str]


class GaitStateMachine:
    """The state machine used to make sure that only valid transitions will be made.

    Args:
        gait_selection (GaitSelection): the gait selection node, loaded gaits to build graph from
        trajectory_scheduler (TrajectoryScheduler): scheduler interface for scheduling trajectories
    Attributes:
        logger (Logger): used to log to the terminal
        update_timer (???): ??? TODO: Add docs
        timer_period (float): state machine runs at intervals of timer_period
        current_state_pub (Publisher): publishes CurrentState message
        current_gait_pub (Publisher): publishes CurrentGait message
        error_sub (Subscriber): subscribes to the /march/error topic

        _gait_selection (GaitSelection): the gait selection node, loaded gaits to build graph from
        _trajectory_scheduler (TrajectoryScheduler): scheduler interface for scheduling trajectories
        _input (StateMachineInput): ??? TODO: Add docs
        _transition_callbacks (List[???]): ??? TODO: Add docs
        _gait_callbacks (List[???]): ??? TODO: Add docs
        _stop_accepted_callbacks (List[???]): ??? TODO: Add docs
        _gait_graph (GaitGraph): ??? TODO: Add docs
        _current_state (State): Either an EdgePosition or a string representing the active gait
        _current_gait (???): The current gait
        _shutdown_requested (bool): whether the state machine should shut down
        _should_stop (bool): whether the gait should be stopped
        _is_stopping (bool): whether the gait is currently stopping
        _right_foot_on_ground (bool): whether the right foot is on the ground or not
        _left_foot_on_ground (bool): whether the left foot is on the ground or not
        _force_right_foot (float): amount of force on right foot
        _force_left_foot (float): amount of force on left foot
        _right_pressure_sub (Subscriber): subscribes to /march/sensor/right_pressure_sole
        _left_pressure_sub (Subscriber): subscribes to /march/sensor/left_pressure_sole
        _get_possible_gaits_client (Service): service to get possible gaits from gait_selection
    """

    UNKNOWN = "unknown"

    def __init__(self, gait_selection: GaitSelection, trajectory_scheduler: TrajectoryScheduler):
        """Generates a state machine from given gaits and resets it to UNKNOWN state.

        In order to start the state machine see `run`.
        """
        self._gait_selection = gait_selection
        self._trajectory_scheduler = trajectory_scheduler
        self.logger = Logger(self._gait_selection, __class__.__name__)

        self._input = StateMachineInput(gait_selection)

        self._transition_callbacks = []
        self._gait_callbacks = []
        self._stop_accepted_callbacks = []

        self._gait_graph = GaitGraph(self._gait_selection)
        self._gait_graph.generate_graph()

        # Current state is either an EdgePositions or a string representing the active gait
        self._current_state: State = UnknownEdgePosition()
        self._current_gait = None
        self._shutdown_requested = False

        # Boolean flag that indicates that the gait should stop
        self._should_stop = False

        # Boolean flag to not execute a stop when the gait is already stopping
        self._is_stopping = False

        self.update_timer = None

        self.timer_period = self._gait_selection.get_parameter("timer_period").get_parameter_value().double_value

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

        self._right_foot_on_ground = True
        self._left_foot_on_ground = True
        self._force_right_foot = 0
        self._force_left_foot = 0
        self._right_pressure_sub = self._gait_selection.create_subscription(
            msg_type=ContactsState,
            topic="/march/sensor/right_pressure_sole",
            callback=lambda msg: self._update_foot_on_ground_cb(Side.right, msg),
            callback_group=ReentrantCallbackGroup(),
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )
        self._left_pressure_sub = self._gait_selection.create_subscription(
            msg_type=ContactsState,
            topic="/march/sensor/left_pressure_sole",
            callback=lambda msg: self._update_foot_on_ground_cb(Side.left, msg),
            callback_group=ReentrantCallbackGroup(),
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )

        self._get_possible_gaits_client = self._gait_selection.create_service(
            srv_type=PossibleGaits,
            srv_name="/march/gait_selection/get_possible_gaits",
            callback=self._possible_gaits_cb,
            callback_group=ReentrantCallbackGroup(),
        )

        self.add_transition_callback(self._current_state_cb)
        self.add_gait_callback(self._current_gait_cb)
        self.logger.debug("Initialized state machine")

    def _is_idle(self):
        return isinstance(self._current_state, EdgePosition)

    def _update_foot_on_ground_cb(self, side: Side, msg: ContactsState) -> None:
        """Update the status of the feet on ground based on pressure sole data.

        This is currently decided based on the force in simulation, but the numbers
        for this will be updated when range of real pressure soles is known.

        Args:
            side (Side): Enum for right or left side
            msg (ContactsState): message published on /march/sensor/[left/right]_pressure_sole
        """
        if len(msg.states) > 0:
            force = sum(state.total_wrench.force.z for state in msg.states)

            # Assign force to specific foot
            if side is Side.right:
                self._force_right_foot = force
            else:
                self._force_left_foot = force

        # If there are no contacts, change foot on ground to False
        elif len(msg.states) == 0:
            if side is Side.right:
                self._force_right_foot = 0
            else:
                self._force_left_foot = 0

    def _possible_gaits_cb(self, request, response: PossibleGaits) -> PossibleGaits:
        """Standard callback for the get possible gaits service.

        Args:
            request: Necessary for service
            response (PossibleGaits): PossibleGaits message to return at service request
        """
        response.gaits = self.get_possible_gaits()
        return response

    def _current_state_cb(self, state: State) -> None:
        """Standard transition callback for when current state changes.

        Publishes the current state. More callbacks can be added using add_transition_callback.

        Args:
            state (State): message passed on through callback
        """
        if self._is_idle():
            state_type = CurrentState.IDLE
            state_name = self._gait_graph.get_name_of_position(self._current_state)

        else:
            state_type = CurrentState.GAIT
            state_name = self._current_state

        self.current_state_pub.publish(
            CurrentState(
                header=Header(stamp=self._gait_selection.get_clock().now().to_msg()),
                state=state_name,
                state_type=state_type,
            )
        )

    def _current_gait_cb(
        self,
        gait_name: str,
        subgait_name: str,
        version: str,
        duration: Duration,
        gait_type: str,
    ) -> None:
        """Standard callback when gait changes, publishes the current gait.

        More callback can be added using add_gait_callback.

        Args:
            gait_name (str): name of the gait
            subgait_name (str): name of the subgait
            version (str): version of the subgait
            duration (Duration): duration of the subgait
            gait_type (str): type of gait, for example walk_like or sit_like
        """
        self.logger.debug(f"Current subgait updated to {subgait_name}")
        self.current_gait_pub.publish(
            CurrentGait(
                header=Header(stamp=self._gait_selection.get_clock().now().to_msg()),
                gait=gait_name,
                subgait=subgait_name,
                version=version,
                duration=duration.to_msg(),
                gait_type=gait_type,
            )
        )

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

    def get_possible_gaits(self) -> Union[List, set]:
        """Returns possible names of gaits that can be executed.

        Returns:
            Union[List, set]: Set of names, or empty list when a gait is executing.
        """
        if self._is_idle():
            return self._gait_graph.possible_gaits_from_idle(self._current_state)
        else:
            return []

    def add_transition_callback(self, cb) -> None:
        """Adds a callback function that will be called when a transition to a state happens.

        The given method should be running as shortly as possible, since they
        will be called from within the main loop.

        TODO: Add cb type
        Args:
            cb: method that accepts a name of the state and a boolean if it is an idle state.

        """
        self._add_callback(self._transition_callbacks, cb)

    def add_gait_callback(self, cb) -> None:
        """Adds a callback function that will be called when a trajectory of a gait is being scheduled.

        The given method should be running as shortly as possible, since they
        will be called from within the main loop.

        TODO: Add cb type
        Args:
            cb: Callable method that accepts 5 args: gait name, subgait name, version, duration and gait type.
        """
        self._add_callback(self._gait_callbacks, cb)

    def add_stop_accepted_callback(self, cb) -> None:
        """Adds a callback function that will be called when a gait accepts the stop command.

        The given method should be running as shortly as possible, since they
        will be called from within the main loop.

        TODO: Add cb type
        Args:
            cb: Callable method that accepts no arguments and returns None.
        """
        self._add_callback(self._stop_accepted_callbacks, cb)

    def run(self) -> None:
        """Runs the state machine until shutdown is requested."""
        self.update_timer = self._gait_selection.create_timer(
            timer_period_sec=self.timer_period,
            callback=self.update,
        )

    def update(self) -> None:
        """Updates the current state based on the elapsed time, after the state machine is started.

        This function is called every timer period.
        """
        if not self._shutdown_requested:
            if self._input.unknown_requested():
                self._input.gait_accepted()
                self._transition_to_unknown()
                self._input.gait_finished()
                self._call_transition_callbacks()
                self._current_gait = None
                self._trajectory_scheduler.reset()

            if self._is_idle():
                self._process_idle_state()
            else:
                self._process_gait_state()
        else:
            self.update_timer.cancel()

    def request_shutdown(self, msg: Optional[str] = None) -> None:
        """Requests shutdown, which will terminate the state machine as soon as possible.

        Args:
            msg (:obj: str, optional): message to log with shutdown request, if not specified will be
                "Shutdown requested"
        """
        base_msg = "Shutdown requested"
        if msg is not None:
            base_msg += ": " + msg
        self.logger.fatal(base_msg)
        self._shutdown_requested = True
        shutdown_system()

    def stop_gait(self) -> None:
        """Requests a stop from the current executing gait, but keeps the state machine running."""
        if not self._is_idle() and not self._is_stopping:
            self._should_stop = True

    def check_correct_foot_pressure(self) -> bool:
        """Check if the pressure is placed on the foot opposite to the subgait starting foot.

        If not, issue a warning. This will only be checked when transitioning from idle to gait state

        Returns:
            bool: True if pressure is on the correct foot, else False
        """
        if self._current_gait is not None:
            if "right" in self._current_gait.subgait_name and self._force_right_foot > self._force_left_foot:
                self.logger.warn("Incorrect pressure placement, place pressure on left foot")
                return False
            if "left" in self._current_gait.subgait_name and self._force_left_foot > self._force_right_foot:
                self.logger.warn("Incorrect pressure placement, place pressure on right foot")
                return False

        return True

    def _process_idle_state(self) -> None:
        """If the current state is idle, this function processes input for what to do next."""
        self._handle_input()
        if self._input.gait_requested():
            gait_name = self._input.gait_name()
            self.logger.info(f"Requested gait `{gait_name}`")
            gait = self._gait_selection._gaits.get(gait_name)
            if (
                    gait is not None
                    and gait_name in self._gait_graph.possible_gaits_from_idle(self._current_state)
                    or gait_name
                    == [
                        "dynamic_walk",
                        "dynamic_step_and_close",
                        "dynamic_step",
                        "dynamic_close",
                    ]
            ):
                if (
                        isinstance(gait.starting_position, DynamicEdgePosition)
                        and gait.starting_position != self._current_state
                ):
                    self.logger.warn(
                        f"The gait {gait_name} does not have the correct dynamic "
                        f"starting position, should be {self._current_state}, but was "
                        f"{gait.starting_position}"
                    )
                    self._input.gait_rejected()
                    return
                self._current_state = gait_name
                self._should_stop = False
                self._input.gait_accepted()
                self._call_transition_callbacks()
                self.logger.info(f"Accepted gait `{gait_name}`")
            else:
                self._input.gait_rejected()
                self.logger.info(f"Cannot execute gait `{gait_name}` from idle state `{self._current_state}`")

    def _process_gait_state(self) -> None:
        """Processes the current state when there is a gait happening.

        Schedules the next subgait if there is no trajectory happening or
        finishes the gait if it is done.
        """
        now = self._gait_selection.get_clock().now()
        if self._current_gait is None:
            self._current_gait = self._gait_selection._gaits[self._current_state]
            self._previous_gait = self._current_gait

            self.logger.info(f"Executing gait `{self._current_gait.name}`")
            if self._current_gait.first_subgait_can_be_scheduled_early:
                gait_update = self._current_gait.start(now, self._gait_selection._first_subgait_delay)
            else:
                gait_update = self._current_gait.start(now)

            if gait_update == GaitUpdate.empty():
                self._input.gait_finished()
                # Find the start position of the current gait, to go back to idle.
                self._current_state = self._current_gait.starting_position
                self._current_gait = None
                self.logger.info(
                    f"Starting the gait returned "
                    f"no trajectory, going back to idle state "
                    f"{self._gait_graph.get_name_of_position(self._current_state)}"
                )
                return

            if not self.check_correct_foot_pressure():
                self.logger.debug(
                    f"Foot forces when incorrect pressure warning was issued: "
                    f"left={self._force_left_foot}, right={self._force_right_foot}"
                )
            self._process_gait_update(gait_update)

        if self._trajectory_scheduler.failed():
            self._trajectory_scheduler.reset()
            self._current_gait.end()
            self._current_gait = None
            self._transition_to_unknown()
            self._input.gait_finished()
            return

        self._handle_input()

        if self._current_gait.subsequent_subgaits_can_be_scheduled_early:
            gait_update = self._current_gait.update(now, self._gait_selection._early_schedule_duration)
        else:
            gait_update = self._current_gait.update(now)
        self._process_gait_update(gait_update)

    def _process_gait_update(self, gait_update: GaitUpdate) -> None:
        """Process an incoming GaitUpdate.

        Will call gait callbacks for each new subgait, schedule new trajectories and process finishing of gait.

        Args:
            gait_update (GaitUpdate): GaitUpdate returned by the gait that is currently executing
        """
        # Call gait callback if there is a new subgait
        if gait_update.is_new_subgait:
            self._call_gait_callbacks()

        # Schedule a new trajectory if any
        if gait_update.new_trajectory_command is not None:
            self._trajectory_scheduler.schedule(gait_update.new_trajectory_command)

        # Process finishing of the gait
        if gait_update.is_finished:
            self._current_state = self._current_gait.final_position
            self.final_position_pub.publish(JointState(position=self._current_state.values))
            # To make the half step dynamic gait work, the current state (that is final
            # position) needs to be a position from which the next half step can be started.
            # Therefore, it needs to be added to the idle_transitions dictionary of the
            # gait_graph.
            if (
                    self._current_gait.name == "dynamic_step"
                    and self._current_state not in self._gait_graph._idle_transitions
            ):
                self._gait_graph._idle_transitions[self._current_state] = {"dynamic_step", "dynamic_close"}
            if (
                    self._current_gait.name == "dynamic_step_and_hold"
                    and self._current_state not in self._gait_graph._idle_transitions
            ):
                self._gait_graph._idle_transitions[self._current_state] = {"dynamic_step_and_hold", "dynamic_close"}
            self._current_gait.end()
            self._input.gait_finished()
            self._call_transition_callbacks()
            self._trajectory_scheduler.reset()
            self.logger.info(f"Finished gait `{self._current_gait.name}`")
            self._current_gait = None
            self._is_stopping = False

    def _handle_input(self) -> None:
        """Handles stop and transition input from the input device.

        This input is passed on to the current gait to execute the request.
        """
        if self._is_stop_requested() and not self._is_stopping:
            if (self._previous_gait.name in ["dynamic_step", "dynamic_step_and_hold"]
                    and not isinstance(self._current_state, UnknownEdgePosition)):
                self._current_state = "dynamic_close"
            else:
                self._should_stop = False
                self._is_stopping = True
                if self._current_gait.stop():
                    self.logger.info(f"Gait {self._current_gait.name} responded to stop")
                    self._input.stop_accepted()
                    self._call_callbacks(self._stop_accepted_callbacks)
                else:
                    self.logger.info(f"Gait {self._current_gait.name} does not respond to stop")
                    self._input.stop_rejected()

        if self._input.transition_requested():
            request = self._input.get_transition_request()
            self._input.reset()
            if self._current_gait.transition(request):
                self.logger.info(f"Gait {self._current_gait.name} responded to transition request {request.name}")
            else:
                self.logger.info(
                    f"Gait {self._current_gait.name} does not respond to transition request {request.name}"
                )

    def _call_transition_callbacks(self) -> None:
        """Calls all transition callbacks when the current state changes."""
        self._call_callbacks(self._transition_callbacks, self._current_state)

    def _is_stop_requested(self) -> bool:
        """Returns true if either the input device requested a stop or some other external source requested a stop.

        Returns:
            bool: True if stop is requested, else False
        """
        return self._input.stop_requested() or self._should_stop

    def _call_gait_callbacks(self) -> None:
        """Calls all added gait callbacks when the current gait changes."""
        if self._current_gait is not None:
            self._call_callbacks(
                self._gait_callbacks,
                self._current_gait.name,
                self._current_gait.subgait_name,
                self._current_gait.version,
                self._current_gait.duration,
                self._current_gait.gait_type,
            )

    def _transition_to_unknown(self) -> None:
        """When the unknown button is pressed, this function resets the state machine to unknown state."""
        if self._current_gait is not None:
            self._trajectory_scheduler.send_position_hold()
            self._trajectory_scheduler.cancel_active_goals()
        self._current_state = UnknownEdgePosition()
        self.logger.info("Transitioned to unknown")

    @staticmethod
    def _add_callback(callbacks, cb) -> None:
        """Adds a method to a list if it is callable. TODO: Add docs."""
        if callable(cb):
            callbacks.append(cb)

    @staticmethod
    def _call_callbacks(callbacks, *args) -> None:
        """Calls multiple methods with same set of arguments. TODO: Add docs."""
        for cb in callbacks:
            cb(*args)

"""Author: Marten Haitjema."""

from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.state_machine.state_machine_input import StateMachineInput
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryScheduler
from march_utility.gait.edge_position import UnknownEdgePosition
from march_utility.utilities.logger import Logger


class GaitStateMachineClean:
    """Clean version of the state machine that can only be used with limited gaits."""

    def __init__(self, gait_selection: GaitSelection, trajectory_scheduler: TrajectoryScheduler):
        self._gait_selection = gait_selection
        self._trajectory_scheduler = trajectory_scheduler
        self._logger = Logger(self._gait_selection, __class__.__name__)
        self._input = StateMachineInput(gait_selection)
        self.timer_period = self._gait_selection.get_parameter("timer_period").get_parameter_value().double_value
        self._shutdown_requested = False

    def run(self) -> None:
        """Runs the state machine until shutdown is requested."""
        self.update_timer = self._gait_selection.create_timer(timer_period_sec=self._timer_period, callback=self.update)

    def update(self) -> None:
        """Updates the current state each timer period, after the state machine is started."""
        if not self._shutdown_requested:
            if self._input.unknown_requested():
                self._transition_to_unknown_state()
            elif self._is_idle():
                self._process_idle_state()
            else:
                self._process_gait_state()
        else:
            self.update_timer.cancel()

    def _process_idle_state(self) -> None:
        """If the current state is idle, this function processes input for what to do next."""
        if self._input.gait_requested():
            gait_name = self._input.gait_name()
            self._logger.info(f"Requested gait `{gait_name}`")
            gait = self._gait_selection._gaits.get(gait_name)

    def _handle_input(self) -> None:
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
                    self.logger.info(f"Gait {self._current_gait.name} responded to stop")
                    self._input.stop_accepted()
                    self._call_callbacks(self._stop_accepted_callbacks)
                else:
                    self.logger.info(f"Gait {self._current_gait.name} does not respond to stop")
                    self._input.stop_rejected()

    def _transition_to_unknown_state(self) -> None:
        self._input.gait_accepted()
        self._transition_to_unknown()
        self._input.gait_finished()
        # should publish current state on /march/gait_selection/current_state

    def _transition_to_unknown(self) -> None:
        """When the unknown button is pressed, this function resets the state machine to unknown state."""
        if self._current_gait is not None:
            self._trajectory_scheduler.send_position_hold()
            self._trajectory_scheduler.cancel_active_goals()
        self._current_state = UnknownEdgePosition()
        self._logger.info("Transitioned to unknown")

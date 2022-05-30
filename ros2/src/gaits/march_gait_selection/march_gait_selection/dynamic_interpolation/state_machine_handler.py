"""Author: Marten Haitjema, MVII."""

from typing import Optional
from rclpy.time import Time

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_utility.exceptions.gait_exceptions import WrongStartPositionError
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger


class StateMachineHandler:
    """Class that handles all interaction between DynamicSetpointGait and the GaitStateMachine."""

    def __init__(self, dynamic_setpoint_gait):
        self._dynamic_setpoint_gait = dynamic_setpoint_gait
        self._logger = Logger(self._dynamic_setpoint_gait.gait_selection, __class__.__name__)
        self._start_time_next_command = None
        self._should_stop = False
        self._start_is_delayed = True
        self._scheduled_early = False
        self._end = False

    DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)

    def start(
        self,
        current_time: Time,
        first_subgait_delay: Duration = DEFAULT_FIRST_SUBGAIT_START_DELAY,
    ) -> Optional[GaitUpdate]:
        """Starts the gait. The subgait will be scheduled with the delay given by first_subgait_delay.

        Args:
            current_time (Time): Time at which the subgait will start
            first_subgait_delay (Duration): Delay of first subgait schedule
        Returns:
            GaitUpdate: An optional GaitUpdate containing a TrajectoryCommand if step is feasible
        """
        try:
            self._dynamic_setpoint_gait.reset()
        except WrongStartPositionError as e:
            self._logger.error(e.msg)
            return None
        self._dynamic_setpoint_gait.update_parameters()
        self._start_time_next_command = current_time + first_subgait_delay
        self._dynamic_setpoint_gait._next_command = (
            self._dynamic_setpoint_gait._trajectory_command_handler._get_trajectory_command(
                self._dynamic_setpoint_gait.subgait_id, start=True
            )
        )
        return GaitUpdate.should_schedule_early(self._dynamic_setpoint_gait._next_command)

    DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION = Duration(0)

    def update(
        self,
        current_time: Time,
        early_schedule_duration: Duration = DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION,
    ) -> GaitUpdate:
        """Give an update on the progress of the gait. This function is called every cycle of the gait_state_machine.

        Schedules the first subgait when the delay has passed. Starts scheduling subsequent subgaits when the previous
        subgait is within early scheduling duration and updates the state machine when the next subgait is started.

        Args:
            current_time (Time): Current time
            early_schedule_duration (Duration): Duration that determines how long ahead the next subgait is planned
        Returns:
            GaitUpdate: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
        """
        if self._start_is_delayed:
            if current_time >= self._start_time_next_command:
                return self._update_start_subgait()
            else:
                return GaitUpdate.empty()

        if current_time >= self._start_time_next_command - early_schedule_duration and not self._scheduled_early:
            return self._update_next_subgait_early()

        if current_time >= self._start_time_next_command:
            return self._update_state_machine()

        return GaitUpdate.empty()

    def stop(self) -> bool:
        """Called when the current gait should be stopped."""
        self._dynamic_setpoint_gait._should_stop = True
        self._should_stop = True
        return True

    def end(self) -> None:
        """Called when the gait is finished."""
        self._dynamic_setpoint_gait._next_command = None

    def _update_start_subgait(self) -> GaitUpdate:
        """Update the state machine that the start gait has begun. Updates the time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        self._start_is_delayed = False
        self._update_time_stamps(self._dynamic_setpoint_gait._next_command.duration)

        return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Already schedule the next subgait with the end time of the current subgait as the start time.

        Returns:
            GaitUpdate: a GaitUpdate that is empty or contains a trajectory command
        """
        self._scheduled_early = True
        self._dynamic_setpoint_gait._next_command = self._dynamic_setpoint_gait._set_and_get_next_command()

        if self._dynamic_setpoint_gait._next_command is None:
            return GaitUpdate.empty()

        return GaitUpdate.should_schedule_early(self._dynamic_setpoint_gait._next_command)

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the new subgait has begun. Also updates time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        if self._dynamic_setpoint_gait._next_command is None:
            return GaitUpdate.finished()

        self._update_time_stamps(self._dynamic_setpoint_gait._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()

    def _update_time_stamps(self, next_command_duration: Duration) -> None:
        """Update the starting and end time.

        Args:
            next_command_duration (Duration): duration of the next command to be scheduled
        """
        start_time_previous_command = self._start_time_next_command
        self._start_time_next_command = start_time_previous_command + next_command_duration

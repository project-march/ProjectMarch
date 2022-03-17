""""Author: Marten Haitjema, MVII"""

from rclpy.node import Node
from rclpy.time import Time
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_utility.utilities.duration import Duration


class DynamicSetpointGaitHalfStep(DynamicSetpointGait):
    """*Single single* step gait based on dynamic setpoint gait

    :param gait_selection_node: the gait selection node
    :type gait_selection_node: Node
    """

    def __init__(self, gait_selection_node: Node):
        super().__init__(gait_selection_node)
        self.subgait_id = "right_swing"
        self.gait_name = "dynamic_walk_half_step"

    def _reset(self) -> None:
        """Reset all attributes of the gait"""
        self._should_stop = False
        self._end = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._next_command = None

        self._start_is_delayed = True
        self._scheduled_early = False

    DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION = Duration(0)

    def update(
        self,
        current_time: Time,
        early_schedule_duration: Duration = DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION,
    ) -> GaitUpdate:
        """Give an update on the progress of the gait. This function is called
        every cycle of the gait_state_machine.

        Schedules the first subgait when the delay has passed. Stops after the
        single single step is finished.

        :param current_time: Current time.
        :type current_time: Time
        :param early_schedule_duration: Duration that determines how long ahead the next subgait is planned
        :type early_schedule_duration: Duration

        :return: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
        :rtype: GaitUpdate
        """
        self._current_time = current_time

        if self._start_is_delayed:
            if self._current_time >= self._start_time:
                return self._update_start_subgait()
            else:
                return GaitUpdate.empty()

        if self._current_time >= self._end_time:
            return self._update_state_machine()

        return GaitUpdate.empty()

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the single single
        step has finished. Also switches the subgait_id.

        :returns: a GaitUpdate for the state machine
        :rtype: GaitUpdate
        """
        if not self._trajectory_failed:
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"
        return GaitUpdate.finished()

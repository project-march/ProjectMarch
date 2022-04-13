"""Author: Marten Haitjema, MVII."""

import os
import yaml

from rclpy.time import Time
from typing import Optional
from ament_index_python.packages import get_package_share_path

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.duration import Duration

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition


class DynamicSetpointGaitHalfStep(DynamicSetpointGait):
    """*Single single* step gait based on dynamic setpoint gait.

    Args:
        gait_selection_node (GaitSelection): the gait selection node

    Attributes:
        _position_queue (List[Dict[str, float]]): List containing foot position dictionaries for x, y and z coordinates.
            Defined in _position_queue.yaml
        _queue_index (int): index that is used for getting the correct dictionary out of _position_queue. Increases
            by one for each step.
        _duration_from_yaml (float): duration of the step as specified in _position_queue.yaml
        _use_position_queue (bool): True if _position_queue will be used instead of covid points, else False
    """

    _current_time: Optional[Time]

    def __init__(self, gait_selection_node):
        super().__init__(gait_selection_node)
        self.subgait_id = "right_swing"
        self.gait_name = "dynamic_walk_half_step"

        queue_path = get_package_share_path("march_gait_selection")
        queue_directory = os.path.join(queue_path, "position_queue", "position_queue.yaml")
        with open(queue_directory, "r") as queue_file:
            _position_queue_yaml = yaml.load(queue_file, Loader=yaml.SafeLoader)

        self._use_position_queue = _position_queue_yaml["use_position_queue"]
        if self._use_position_queue:
            self._position_queue = _position_queue_yaml["points"]
        else:
            self._position_queue = None

        self._queue_index = 0
        self._duration_from_yaml = _position_queue_yaml["duration"]

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
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
        """Give an update on the progress of the gait. This function is called every cycle of the gait_state_machine.

        Schedules the first subgait when the delay has passed. Stops after the single single step is finished.

        Args:
            current_time (Time): Current time
            early_schedule_duration (Duration): Duration that determines how long ahead the next subgait is planned
        Returns:
            GaitUpdate: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
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
        """Update the state machine that the single single step has finished. Also switches the subgait_id.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        if not self._trajectory_failed:
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"
        return GaitUpdate.finished()

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on the _position_queue if enabled.

        Args:
            start (:obj: bool, optional): whether it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        if self._start_is_delayed:
            self._end_time = self._start_time

        if stop:
            self._end = True
            self.logger.info("Stopping dynamic gait.")
        else:
            if self._use_position_queue:
                if self._queue_index <= (len(self._position_queue) - 1):
                    self.foot_location = self._get_foot_location_from_queue()
                else:
                    self._end = True
                    stop = True
                    self._queue_index = 0
            else:
                self.foot_location = self._get_foot_location(self.subgait_id)
                stop = self._check_msg_time(self.foot_location)

            self.logger.debug(
                f"Stepping to location ({self.foot_location.point.x}, {self.foot_location.point.y}, "
                f"{self.foot_location.point.z})"
            )

        return self._get_first_feasible_trajectory(start, stop)

    def _get_foot_location_from_queue(self) -> FootPosition:
        """Get FootPosition message from the position queue.

        Returns:
            FootPosition: FootPosition msg with position from queue
        """
        msg = f"Step {self._queue_index + 1} out of {len(self._position_queue)}"
        if self._queue_index == len(self._position_queue) - 1:
            msg += ". Next step will be a close gait."
        self.logger.warn(msg)

        header = Header(stamp=self.gait_selection.get_clock().now().to_msg())
        point = Point(
            x=self._position_queue[self._queue_index]["x"],
            y=self._position_queue[self._queue_index]["y"],
            z=self._position_queue[self._queue_index]["z"],
        )
        self._queue_index += 1

        return FootPosition(header=header, point=point, duration=self._duration_from_yaml)

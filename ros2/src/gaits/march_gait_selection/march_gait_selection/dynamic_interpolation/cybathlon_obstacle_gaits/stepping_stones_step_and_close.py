"""Author: Marten Haitjema, MVII."""

from copy import deepcopy
from typing import Optional

from rclpy.time import Time

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.exceptions.gait_exceptions import (
    WrongStartPositionError,
)
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import Point
from std_msgs.msg import String

PREDETERMINED_FOOT_LOCATIONS = {
    "small_narrow": FootPosition(duration=1.5, processed_point=Point(x=0.55, y=0.03, z=0.44699999999999995)),
    "small_wide": FootPosition(duration=1.5, processed_point=Point(x=0.65, y=0.03, z=0.44699999999999995)),
    "large_narrow": FootPosition(duration=2.0, processed_point=Point(x=0.75, y=0.03, z=0.44699999999999995)),
    "large_wide": FootPosition(duration=2.0, processed_point=Point(x=0.85, y=0.03, z=0.44699999999999995)),
    "step_up_obstacle": FootPosition(duration=1.5, processed_point=Point(x=0.65, y=0.09, z=0.44699999999999995)),
}


class SteppingStonesStepAndClose(DynamicSetpointGaitStepAndClose):
    """Class for doing step and closes on the cybathlon stepping stones obstacle.

    Args:
        gait_selection_node (GaitSelection): the gait selection node

    Attributes:
        _start_from_left_side (bool): whether the gaits start with a left swing
        _use_predetermined_foot_location (bool): whether one of the five predetermined locations will be used
    """

    def __init__(self, gait_selection_node):
        super().__init__(gait_selection_node)
        self._start_from_left_side = False
        self._use_predetermined_foot_location = False
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "stepping_stones_step_and_close"

        self.gait_selection.create_subscription(
            String,
            "/march/step_and_hold/start_side",
            self._set_start_subgait_id,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            String,
            "/march/step_and_hold/step_size",
            self._predetermined_foot_location_callback,
            DEFAULT_HISTORY_DEPTH,
        )

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
            self._reset()
        except WrongStartPositionError as e:
            self.logger.error(e.msg)
            return None

        if self._start_from_left_side:
            self.subgait_id = "left_swing"
        else:
            self.subgait_id = "right_swing"

        # reset _start_from_left_side attribute
        self._start_from_left_side = False

        self.update_parameters()
        self._start_time_next_command = current_time + first_subgait_delay
        self._next_command = self._get_trajectory_command(start=True)
        return GaitUpdate.should_schedule_early(self._next_command)

    DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION = Duration(0)

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on the _position_queue if enabled.

        Args:
            start (Optional[bool]): whether it is a start gait or not, default False
            stop (Optional[bool]): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        if stop:
            self.logger.info("Stopping dynamic gait.")
        else:
            if self._use_predetermined_foot_location:
                self.foot_location = deepcopy(self._predetermined_foot_location)
                self._use_predetermined_foot_location = False
            else:
                try:
                    self.foot_location = self._get_foot_location(self.subgait_id)
                    stop = self._is_foot_location_too_old(self.foot_location)
                    if stop:
                        return None
                except AttributeError:
                    self.logger.info("No FootLocation found. Connect the camera or use simulated points.")
                    self._end = True
                    return None

            if not stop:
                self._publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self.logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        return self._get_first_feasible_trajectory(start, stop)

    def _set_start_subgait_id(self, start_side: String) -> None:
        """Sets the subgait_id to the given start side, if and only if exo is in homestand."""
        try:
            if self.start_position_all_joints == self.home_stand_position_all_joints:
                self.subgait_id = start_side.data
                if self.subgait_id == "left_swing":
                    self._start_from_left_side = True
                else:
                    self._start_from_left_side = False
                self.logger.info(f"Starting subgait set to {self.subgait_id}")
            else:
                raise WrongStartPositionError(self.home_stand_position_all_joints, self.start_position_all_joints)
        except WrongStartPositionError as e:
            self.logger.warn(f"Can only change start side in home stand position. {e.msg}")

    def _predetermined_foot_location_callback(self, msg: String) -> None:
        """Sets the predetermined foot location to the requested location given by the IPD."""
        self._use_predetermined_foot_location = True
        self._predetermined_foot_location = PREDETERMINED_FOOT_LOCATIONS[msg.data]
        self.logger.info(f"Stepping to stone {msg.data}")

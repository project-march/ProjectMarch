"""Author: Marten Haitjema, MVII."""

from copy import deepcopy
from typing import Optional, Dict

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

from march_gait_selection.dynamic_interpolation.trajectory_command_handlers.trajectory_command_handler_fixed_sizes import (
    TrajectoryCommandHandlerFixedSizes,
)


class SteppingStonesStepAndClose(DynamicSetpointGaitStepAndClose):
    """Class for doing step and closes on the cybathlon stepping stones obstacle.

    Args:
        gait_selection_node (GaitSelection): the gait selection node

    Attributes:
        start_from_left_side (bool): whether the gaits start with a left swing
        use_predetermined_foot_location (bool): whether one of the five predetermined locations will be used
    """

    def __init__(self, gait_selection_node):
        super().__init__(gait_selection_node)
        self.start_from_left_side = False
        self.use_predetermined_foot_location = False
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.trajectory_command_handler = TrajectoryCommandHandlerSteppingStones(self, self._camera_points_handler)
        self.gait_name = "stepping_stones_step_and_close"

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

        if self.start_from_left_side:
            self.subgait_id = "left_swing"
        else:
            self.subgait_id = "right_swing"

        # reset _start_from_left_side attribute
        self.start_from_left_side = False

        self.update_parameters()
        self.start_time_next_command = current_time + first_subgait_delay
        self._next_command = self.trajectory_command_handler.get_trajectory_command(
            self.subgait_id,
            self.start_position_all_joints,
            start=True,
        )
        return GaitUpdate.should_schedule_early(self._next_command)


class TrajectoryCommandHandlerSteppingStones(TrajectoryCommandHandlerFixedSizes):
    """TrajectoryCommandHandler for the stepping stones gait."""

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)

    def get_trajectory_command(
        self, subgait_id: str, start_position_all_joints: Dict[str, float], start=False, stop=False
    ) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on a predetermined foot location.

         Args:
            subgait_id (str): whether it is a right_swing or left_swing
            start_position_all_joints (Dict[str, float]): start joint angles of all joints
            start (bool): whether it is a start gait, default False
            stop (bool): whether it is a stop gait, default False

        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        self.subgait_id = subgait_id
        self.start_position_all_joints = start_position_all_joints

        if stop:
            self._logger.info("Stopping dynamic gait.")
        else:
            if self._gait.use_predetermined_foot_location:
                self.foot_location = deepcopy(self._predetermined_foot_location)
                self._gait.use_predetermined_foot_location = False
            else:
                try:
                    self.foot_location = self._points_handler.get_foot_location(self.subgait_id)
                    stop = self._points_handler.is_foot_location_too_old(self.foot_location)
                    if stop:
                        return None
                except AttributeError:
                    self._logger.info("No FootLocation found. Connect the camera or use simulated points.")
                    self._end = True
                    return None

            if not stop:
                self._points_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self._logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        return self._get_first_feasible_trajectory(start, stop)

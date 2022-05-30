"""Author: Marten Haitjema, MVII."""

from typing import Optional
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError, VelocitySoftLimitError
from march_utility.utilities.logger import Logger
from .trajectory_command_handler import TrajectoryCommandHandler


class DynamicSetpointGaitStepAndClose(DynamicSetpointGait):
    """Single step gait based on dynamic setpoint gait.

    Args:
        gait_selection_node (GaitSelection): the gait selection node
    """

    def __init__(self, gait_selection_node):
        super().__init__(gait_selection_node)
        self._logger = Logger(gait_selection_node, __class__.__name__)
        self.trajectory_command_handler = TrajectoryCommandHandlerStepAndClose(
            gait=self,
            points_handler=self._camera_points_handler,
        )
        self.gait_name = "dynamic_step_and_close"

    def _set_and_get_next_command(self) -> Optional[TrajectoryCommand]:
        """Create the next command. Because it is a single step, this will always be a left_swing and a close gait.

        Returns:
            TrajectoryCommand: A TrajectoryCommand for the next subgait
        """
        if not self._trajectory_failed:
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"

        if self._end:
            # If the gait has ended, the next command should be None
            return None
        else:
            self._end = True
            return self.trajectory_command_handler.get_trajectory_command(
                self.subgait_id, self.start_position_all_joints, stop=True
            )


class TrajectoryCommandHandlerStepAndClose(TrajectoryCommandHandler):
    """Class that creates and validates a trajectory command for a step and close."""

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)
        self._gait = gait
        self._points_handler = points_handler
        self._logger = Logger(self._gait.gait_selection, __class__.__name__)
        self._trajectory_failed = False

    def _can_get_second_step(self, is_final_iteration: bool) -> bool:
        """Tries to create the subgait that is one step ahead, which is a stop gait for step and close.

        If this is not possible, the first subgait should not be executed.

        Args:
            is_final_iteration (bool): True if current iteration equals the maximum amount of iterations
        Returns:
            bool: true if second step close gait can be made.
        """
        start_position = self.dynamic_subgait.get_final_position()
        subgait_id = "right_swing" if self.subgait_id == "left_swing" else "left_swing"
        subgait = self._create_subgait_instance(
            start_position,
            subgait_id,
            start=False,
            stop=True,
        )
        try:
            subgait.get_joint_trajectory_msg(self._gait.add_push_off)
        except (PositionSoftLimitError, VelocitySoftLimitError) as e:
            if is_final_iteration:
                self._logger.warn(f"Close gait is not feasible. {e.msg}")
            return False
        return True

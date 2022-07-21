"""Author: Marten Haitjema, MVII."""

from typing import Optional
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError, VelocitySoftLimitError, GaitError
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)
from march_shared_msgs.msg import FootPosition


class TrajectoryCommandFactoryStepAndClose(TrajectoryCommandFactory):
    """Class that creates and validates a trajectory command for a step and close."""

    def __init__(self, gait, point_handler):
        super().__init__(gait, point_handler)
        self._gait = gait
        self._point_handler = point_handler
        self._logger = gait.node.get_logger().get_child(__class__.__name__)
        self._trajectory_failed = False

    def _check_if_queue_is_not_empty_and_get_foot_location(self) -> Optional[FootPosition]:
        if self.position_queue.qsize() == 1:
            self._logger.warn("Last step of queue. Queue will be reset after this step.")
        elif self.position_queue.empty():
            self._close_gait_and_reset_queue()
        return self._get_foot_location_from_queue()

    def _close_gait_and_reset_queue(self) -> None:
        """Closes the gait and reset the queue after the queue is empty."""
        self.fill_queue()
        self._logger.warn(f"Queue is empty. Resetting queue to {list(self.position_queue.queue)}")

    def _can_get_second_step(self, is_final_iteration: bool) -> bool:
        """Tries to create the subgait that is one step ahead, which is a stop gait for step and close.

        If this is not possible, the first subgait should not be executed.

        Args:
            is_final_iteration (bool): True if current iteration equals the maximum amount of iterations
        Returns:
            bool: true if second step close gait can be made.
        """
        start_position = self.dynamic_step.get_final_position()
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
                msg = f"Close gait is not feasible. {e}"
                raise GaitError(msg)
            return False
        return True

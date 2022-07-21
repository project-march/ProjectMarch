"""Author: Marten Haitjema, MVII."""

from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)


class TrajectoryCommandFactoryStepAndClose(TrajectoryCommandFactory):
    """Class that creates and validates a trajectory command for a step and close."""

    def __init__(self, gait, point_handler):
        super().__init__(gait, point_handler)
        self._gait = gait
        self._point_handler = point_handler
        self._logger = gait.node.get_logger().get_child(__class__.__name__)
        self._trajectory_failed = False

    def _can_get_second_step(self) -> bool:
        """Tries to create the subgait that is one step ahead, which is a stop gait for step and close.

        If this is not possible, the first subgait should not be executed.

        Returns:
            bool: true if second step close gait can be made.
        """
        start_position = self.dynamic_step.get_final_position()
        subgait_id = "right_swing" if self.subgait_id == "left_swing" else "left_swing"
        return self._create_trajectory_command(start_position, subgait_id, start=False, stop=True) is not None

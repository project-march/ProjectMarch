"""Author: Marten Haitjema, MVII."""

from typing import Optional, Dict

from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_walk import (
    DynamicGaitWalk,
)
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand

from march_utility.gait.edge_position import EdgePosition
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory_step_and_close import (
    TrajectoryCommandFactoryStepAndClose,
)


class DynamicGaitStepAndClose(DynamicGaitWalk):
    """Single step gait based on dynamic setpoint gait.

    Args:
        node (Node): the gait selection node
    """

    def __init__(self, node, positions: Dict[str, EdgePosition]):
        super().__init__(node, positions)
        self._logger = node.get_logger().get_child(__class__.__name__)
        self.trajectory_command_factory = TrajectoryCommandFactoryStepAndClose(
            gait=self,
            points_handler=self._points_handler,
        )
        self.gait_name = "dynamic_step_and_close"

    def _set_and_get_next_command(self) -> Optional[TrajectoryCommand]:
        """Create the next command. Because it is a single step, this will always be a left_swing and a close gait.

        Returns:
            TrajectoryCommand: A TrajectoryCommand for the next subgait
        """
        if not self.trajectory_command_factory.has_trajectory_failed():
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"

        if self._end:
            # If the gait has ended, the next command should be None
            return None
        else:
            self._end = True
            return self.trajectory_command_factory.get_trajectory_command(
                self.subgait_id, self.start_position_all_joints, stop=True
            )

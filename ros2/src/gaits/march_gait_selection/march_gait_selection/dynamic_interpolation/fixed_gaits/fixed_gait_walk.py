"""Author: Marten Haitjema, MVII."""

from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_walk import DynamicGaitWalk
from march_gait_selection.dynamic_interpolation.camera_point_handlers.simulated_points_handler import (
    SimulatedPointsHandler,
)
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)


class FixedGaitWalk(DynamicGaitWalk):
    """Class to have a setpoint gait with a fixed step distance."""

    def __init__(self, node):
        super().__init__(node)
        self.gait_name = "fixed_walk"
        self._points_handler = SimulatedPointsHandler(gait=self)
        self.trajectory_command_factory = TrajectoryCommandFactory(
            gait=self,
            points_handler=self._points_handler,
        )

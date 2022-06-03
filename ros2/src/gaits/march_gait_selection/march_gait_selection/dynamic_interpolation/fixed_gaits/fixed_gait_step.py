"""Author: Marten Haitjema, MVII."""

from rclpy.node import Node
from typing import Dict

from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_step import DynamicGaitStep
from march_gait_selection.dynamic_interpolation.camera_point_handlers.simulated_points_handler import (
    SimulatedPointsHandler,
)
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)

from march_utility.gait.edge_position import EdgePosition


class FixedGaitStep(DynamicGaitStep):
    """Class to have a setpoint gait step and close with a fixed step distance."""

    def __init__(self, node: Node, positions: Dict[str, EdgePosition]):
        super().__init__(node, positions)
        self.gait_name = "fixed_step"
        self._points_handler = SimulatedPointsHandler(gait=self)
        self.trajectory_command_factory = TrajectoryCommandFactory(
            gait=self,
            points_handler=self._points_handler,
        )

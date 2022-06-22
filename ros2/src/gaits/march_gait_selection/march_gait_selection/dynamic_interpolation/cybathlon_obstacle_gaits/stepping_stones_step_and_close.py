"""Author: Marten Haitjema, MVII."""

from typing import Optional
from rclpy.time import Time
from rclpy.node import Node

from march_gait_selection.dynamic_interpolation.camera_point_handlers.points_handler import PointsHandler
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_step_and_close import DynamicGaitStepAndClose
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_utility.exceptions.gait_exceptions import (
    WrongStartPositionError,
)
from march_utility.utilities.duration import Duration

from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory_stepping_stones import (
    TrajectoryCommandFactorySteppingStones,
)


class SteppingStonesStepAndClose(DynamicGaitStepAndClose):
    """Class for doing step and closes on the cybathlon stepping stones obstacle.

    Args:
        node (Node): the gait node

    Attributes:
        start_from_left_side (bool): whether the gaits start with a left swing
        use_predetermined_foot_location (bool): whether one of the five predetermined locations will be used
    """

    def __init__(self, name: str, node: Node, points_handler: PointsHandler):
        super().__init__(name, node, points_handler)
        self.node = node
        self.start_from_left_side = False
        self.use_predetermined_foot_location = False
        self._logger = node.get_logger().get_child(__class__.__name__)
        self.trajectory_command_factory = TrajectoryCommandFactorySteppingStones(self, self._points_handler)
        self.gait_name = name

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
            self._logger.error(e.msg)
            return None

        if self.start_from_left_side:
            self.subgait_id = "left_swing"
        else:
            self.subgait_id = "right_swing"

        # reset _start_from_left_side attribute
        self.start_from_left_side = False

        self.update_parameters()
        self.start_time_next_command = current_time + first_subgait_delay
        self._next_command = self.trajectory_command_factory.get_trajectory_command(
            self.subgait_id,
            self.start_position_all_joints,
            start=True,
        )
        return GaitUpdate.should_schedule_early(self._next_command)

"""Author: Marten Haitjema, MVII."""

from typing import Optional

from rclpy.node import Node
from rclpy.time import Time

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import DynamicSetpointGait
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_hold import (
    END_POSITION_RIGHT,
    END_POSITION_LEFT,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_utility.utilities.duration import Duration

from march_shared_msgs.msg import FootPosition, CurrentGait

from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH


class DynamicSetpointGaitClose(DynamicSetpointGait):
    """Class for closing the gait after a dynamic step."""

    def __init__(self, gait_selection_node: Node):
        super().__init__(gait_selection_node)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "dynamic_close"

        gait_selection_node.create_subscription(
            CurrentGait,
            "/march/gait_selection/current_gait",
            self._set_subgait_id,
            DEFAULT_HISTORY_DEPTH,
        )
        gait_selection_node.create_subscription(
            FootPosition,
            "/march/chosen_foot_position/right",
            self._set_last_foot_position,
            DEFAULT_HISTORY_DEPTH,
        )
        gait_selection_node.create_subscription(
            FootPosition,
            "/march/chosen_foot_position/left",
            self._set_last_foot_position,
            DEFAULT_HISTORY_DEPTH,
        )

    @property
    def gait_type(self) -> Optional[str]:
        """Returns the type of gait, for example 'walk_like' or 'sit_like'."""
        prefix = "right" if self.subgait_id == "right_swing" else "left"
        return prefix + "_close"

    def _set_subgait_id(self, current_gait: CurrentGait) -> None:
        """Sets the subgait_id based on the gait that has been previously executed."""
        previous_subgait_id = current_gait.subgait
        self.subgait_id = "left_swing" if previous_subgait_id == "right_swing" else "right_swing"

    def _set_last_foot_position(self, foot_position: FootPosition) -> None:
        """Set the foot position to use for the close gait to the last used foot position."""
        self.foot_location = foot_position

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
        if self.start_position_actuating_joints == self.home_stand_position_actuating_joints:
            self.logger.warn("Already in home stand position.")
            return None
        elif self.start_position_all_joints == END_POSITION_RIGHT:
            self.subgait_id = "right_swing"
        elif self.start_position_all_joints == END_POSITION_LEFT:
            self.subgait_id = "left_swing"

        self.update_parameters()
        self._start_time_next_command = current_time + first_subgait_delay
        self._next_command = self._get_trajectory_command(stop=True)
        return GaitUpdate.should_schedule_early(self._next_command)

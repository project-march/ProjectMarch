"""Author: Marten Haitjema, MVII."""

from typing import Optional

from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import DynamicSetpointGait
from march_gait_selection.dynamic_interpolation.cybathlon_obstacle_gaits.dynamic_setpoint_gait_step_and_hold import (
    END_POSITION_RIGHT,
    END_POSITION_LEFT,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

from march_shared_msgs.msg import CurrentGait, FootPosition

from .trajectory_command_handler import TrajectoryCommandHandler
from .camera_points_handler import CameraPointsHandler


class DynamicSetpointGaitClose(DynamicSetpointGait):
    """Class for closing the gait after a dynamic step."""

    def __init__(self, gait_selection_node: Node):
        super().__init__(gait_selection_node)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        camera_points_handler = CameraPointsHandler(self)
        self.trajectory_command_handler = TrajectoryCommandHandlerStep(self, camera_points_handler)
        self.gait_name = "dynamic_close"

        gait_selection_node.create_subscription(
            CurrentGait,
            "/march/gait_selection/current_gait",
            self._set_subgait_id,
            DEFAULT_HISTORY_DEPTH,
        )
        gait_selection_node.create_subscription(
            JointState,
            "/march/step/final_position",
            self._update_start_position_idle_state,
            DEFAULT_HISTORY_DEPTH,
        )
        gait_selection_node.create_subscription(
            JointState,
            "/march/step_and_hold/final_position",
            self._update_start_position_idle_state,
            DEFAULT_HISTORY_DEPTH,
        )
        self._final_position_pub = gait_selection_node.create_publisher(
            JointState,
            "/march/close/final_position",
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
            return GaitUpdate.empty()
        elif self.start_position_all_joints == END_POSITION_RIGHT:
            self.subgait_id = "right_swing"
        elif self.start_position_all_joints == END_POSITION_LEFT:
            self.subgait_id = "left_swing"

        self.update_parameters()
        self.start_time_next_command = current_time + first_subgait_delay
        self._next_command = self.trajectory_command_handler.get_trajectory_command(
            self.subgait_id,
            self.start_position_all_joints,
            stop=True,
        )
        return GaitUpdate.should_schedule_early(self._next_command)

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the new subgait has begun. Also updates time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        self._final_position_pub.publish(
            JointState(position=self.trajectory_command_handler.dynamic_subgait.get_final_position().values())
        )
        if self._next_command is None:
            return GaitUpdate.finished()

        self._update_time_stamps(self._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()


class TrajectoryCommandHandlerStep(TrajectoryCommandHandler):
    """TrajectoryCommandHandler class but with a hard coded foot location for the close gait."""

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)
        self._gait = gait
        self._points_handler = points_handler
        self._logger = Logger(self._gait.gait_selection, __class__.__name__)
        self._trajectory_failed = False
        # TODO: remove hardcoded foot location after bug in stop gait is fixed.
        self.foot_location = FootPosition(duration=1.5, processed_point=Point(x=0.5, y=0.03, z=0.446))

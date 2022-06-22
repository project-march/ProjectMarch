"""Author: Marten Haitjema, MVII."""

from typing import Optional
from rclpy.node import Node
from rclpy.time import Time

from march_gait_selection.dynamic_interpolation.camera_point_handlers.points_handler import PointsHandler
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_walk import DynamicGaitWalk
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory_close import (
    TrajectoryCommandFactoryClose,
)

from march_utility.utilities.utility_functions import (
    STEPPING_STONES_END_POSITION_RIGHT,
    STEPPING_STONES_END_POSITION_LEFT,
)
from march_utility.utilities.duration import Duration
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

from sensor_msgs.msg import JointState
from march_shared_msgs.msg import CurrentGait


class DynamicGaitClose(DynamicGaitWalk):
    """Class for closing the gait after a dynamic step."""

    subgait_id: str
    start_time_next_command: Time

    def __init__(self, name: str, node: Node, points_handler: PointsHandler):
        super().__init__(name, node, points_handler)
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._points_handler = points_handler
        self.trajectory_command_factory = TrajectoryCommandFactoryClose(self, self._points_handler)
        self.gait_name = name

        self.node.create_subscription(
            CurrentGait,
            "/march/gait_selection/current_gait",
            self._set_subgait_id,
            DEFAULT_HISTORY_DEPTH,
        )
        self.node.create_subscription(
            JointState,
            "/march/step/final_position",
            self._update_start_position_idle_state,
            DEFAULT_HISTORY_DEPTH,
        )
        self.node.create_subscription(
            JointState,
            "/march/step_and_hold/final_position",
            self._update_start_position_idle_state,
            DEFAULT_HISTORY_DEPTH,
        )
        self._final_position_pub = self.node.create_publisher(
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
            self._logger.warn("Already in home stand position.")
            return GaitUpdate.empty()
        elif self.start_position_all_joints == STEPPING_STONES_END_POSITION_RIGHT:
            self.subgait_id = "right_swing"
        elif self.start_position_all_joints == STEPPING_STONES_END_POSITION_LEFT:
            self.subgait_id = "left_swing"

        self.update_parameters()
        self.start_time_next_command = current_time + first_subgait_delay
        self._next_command = self.trajectory_command_factory.get_trajectory_command(
            self.subgait_id,
            self.start_position_all_joints,
            stop=True,
        )
        return GaitUpdate.should_schedule_early(self._next_command)

    def _update_start_position_idle_state(self, joint_state: JointState) -> None:
        """Update the start position of the next subgait to be the last position of the previous gait."""
        for i, name in enumerate(self.all_joint_names):
            self.start_position_all_joints[name] = joint_state.position[i]
        self.start_position_actuating_joints = {
            name: self.start_position_all_joints[name] for name in self.actuating_joint_names
        }

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the new subgait has begun. Also updates time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        self._final_position_pub.publish(
            JointState(position=self.trajectory_command_factory.dynamic_step.get_final_position().values())
        )
        if self._next_command is None:
            return GaitUpdate.finished()

        self._update_time_stamps(self._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()

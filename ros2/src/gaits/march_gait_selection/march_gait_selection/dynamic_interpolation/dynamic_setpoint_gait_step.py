"""Author: Marten Haitjema, MVII."""

from queue import Queue
from rclpy.node import Node
from rclpy.time import Time
from typing import Optional
from sensor_msgs.msg import JointState

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.dynamic_interpolation.trajectory_command_handlers.trajectory_command_handler_queue import (
    TrajectoryCommandHandlerQueue,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

from march_shared_msgs.msg import GaitInstruction


class DynamicSetpointGaitStep(DynamicSetpointGait):
    """Step gait based on dynamic setpoint gait.

    Args:
        gait_selection_node (GaitSelection): the gait selection node
    """

    _current_time: Optional[Time]
    _use_position_queue: bool

    def __init__(self, gait_selection_node: Node):
        super().__init__(gait_selection_node)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.trajectory_command_handler = TrajectoryCommandHandlerQueue(
            gait=self, points_handler=self._camera_points_handler
        )
        self.subgait_id = "right_swing"
        self.gait_name = "dynamic_step"

        self.gait_selection.create_subscription(
            JointState,
            "/march/close/final_position",
            self._update_start_position_idle_state,
            DEFAULT_HISTORY_DEPTH,
        )
        self._final_position_pub = self.gait_selection.create_publisher(
            JointState,
            "/march/step/final_position",
            DEFAULT_HISTORY_DEPTH,
        )

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        if self.start_position_actuating_joints == self.home_stand_position_actuating_joints:
            self.subgait_id = "right_swing"

        self._should_stop = False
        self._end = False
        self.start_time_next_command = None
        self._current_time = None
        self._next_command = None
        self._has_gait_started = False
        self._scheduled_early = False

    DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION = Duration(0)

    def update(
        self,
        current_time: Time,
        early_schedule_duration: Duration = DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION,
    ) -> GaitUpdate:
        """Give an update on the progress of the gait. This function is called every cycle of the gait_state_machine.

        Schedules the first subgait when the delay has passed. Stops after the single single step is finished.

        Args:
            current_time (Time): Current time
            early_schedule_duration (Duration): Duration that determines how long ahead the next subgait is planned
        Returns:
            GaitUpdate: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
        """
        if current_time >= self.start_time_next_command and not self._has_gait_started:
            self._has_gait_started = True
            return self._update_start_subgait()

        elif current_time >= self.start_time_next_command and self._has_gait_started:
            self._scheduled_early = True
            self._final_position_pub.publish(
                JointState(position=self.trajectory_command_handler.dynamic_subgait.get_final_position().values())
            )
            return self._update_state_machine()

        else:
            return GaitUpdate.empty()

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the single single step has finished. Also switches the subgait_id.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        if not self.trajectory_command_handler.has_trajectory_failed():
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"

        if self._end:
            self.subgait_id = "right_swing"

        return GaitUpdate.finished()

    def _callback_force_unknown(self, msg: GaitInstruction) -> None:
        """Resets the subgait_id, _trajectory_failed and position_queue after a force unknown.

        Args:
            msg (GaitInstruction): the GaitInstruction message that may contain a force unknown
        """
        if msg.type == GaitInstruction.UNKNOWN:
            self._set_start_position_to_home_stand()
            self.subgait_id = "right_swing"
            self.trajectory_command_handler.set_trajectory_failed_false()
            self.position_queue = Queue()
            self.trajectory_command_handler.fill_queue()

"""Author: Marten Haitjema, MVII."""

from rclpy.node import Node
from sensor_msgs.msg import JointState

from march_gait_selection.dynamic_interpolation.point_handlers.point_handler import PointHandler
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory_step_and_hold import (
    TrajectoryCommandFactoryStepAndHold,
)
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_step_and_close import DynamicGaitStepAndClose
from march_gait_selection.state_machine.gait_update import GaitUpdate

from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.utility_functions import (
    STEPPING_STONES_END_POSITION_RIGHT,
    STEPPING_STONES_END_POSITION_LEFT,
)

from march_shared_msgs.msg import GaitInstruction


class DynamicGaitStepAndHold(DynamicGaitStepAndClose):
    """Class for stepping to a hold position firstly, and to the desired position secondly."""

    _use_position_queue: bool

    def __init__(self, name: str, node: Node, point_handler: PointHandler):
        self.subgait_id = "right_swing"
        self.use_predetermined_foot_location = False
        self.start_from_left_side = False
        super().__init__(name, node, point_handler)
        self.trajectory_command_factory = TrajectoryCommandFactoryStepAndHold(self, self._point_handler)
        self._logger = node.get_logger().get_child(__class__.__name__)
        self.gait_name = name

        self.node.create_subscription(
            JointState,
            "/march/close/final_position",
            self._update_start_position_idle_state,
            DEFAULT_HISTORY_DEPTH,
        )
        self._final_position_pub = self.node.create_publisher(
            JointState,
            "/march/step_and_hold/final_position",
            DEFAULT_HISTORY_DEPTH,
        )

    @property
    def requires_dynamic_stop(self) -> bool:
        """Return whether this gait needs a dynamic stop.

        This means that the gait does not end in home_stand, but in another random (dynamic) position.
        """
        return True

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        self._should_stop = False
        self._end = False

        self._start_time_next_command = None
        self._current_time = None

        self._next_command = None
        self._has_gait_started = False

        self._scheduled_early = False

        if (
            self.start_position_all_joints == self.home_stand_position_all_joints and not self.start_from_left_side
        ) or (self.start_position_all_joints == STEPPING_STONES_END_POSITION_RIGHT):
            self.subgait_id = "right_swing"
        elif self.start_position_all_joints == STEPPING_STONES_END_POSITION_LEFT:
            self.subgait_id = "left_swing"
        else:
            self.subgait_id = "left_swing"

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

    def _callback_force_unknown(self, msg: GaitInstruction) -> None:
        """Reset start position to home stand after force unknown.

        Args:
            msg (GaitInstruction): message containing a gait_instruction from the IPD
        """
        if msg.type == GaitInstruction.UNKNOWN:
            self._set_start_position_to_home_stand()
            self.use_predetermined_foot_location = False
            self.subgait_id = "right_swing"
            self.trajectory_command_factory.set_trajectory_failed_false()

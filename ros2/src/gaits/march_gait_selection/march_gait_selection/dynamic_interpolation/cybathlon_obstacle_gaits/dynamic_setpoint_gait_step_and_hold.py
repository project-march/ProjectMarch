"""Author: Marten Haitjema, MVII."""

from copy import copy, deepcopy
from typing import Dict, Optional
from rclpy.node import Node
from sensor_msgs.msg import JointState

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.dynamic_interpolation.trajectory_command_handlers.trajectory_command_handler_fixed_sizes import (
    TrajectoryCommandHandlerFixedSizes,
)
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger
from march_utility.utilities.utility_functions import get_position_from_yaml

from march_shared_msgs.msg import GaitInstruction

END_POSITION_RIGHT = get_position_from_yaml("stand")
END_POSITION_RIGHT = dict.fromkeys(END_POSITION_RIGHT, 0)
END_POSITION_LEFT = copy(END_POSITION_RIGHT)

END_POSITION_RIGHT["right_knee"] = 1
END_POSITION_LEFT["left_knee"] = 1


class DynamicSetpointGaitStepAndHold(DynamicSetpointGaitStepAndClose):
    """Class for stepping to a hold position firstly, and to the desired position secondly."""

    _use_position_queue: bool

    def __init__(self, gait_selection_node: Node):
        self.subgait_id = "right_swing"
        self.use_predetermined_foot_location = False
        self.start_from_left_side = False
        super().__init__(gait_selection_node)
        self.trajectory_command_handler = TrajectoryCommandHandlerStepAndHold(self, self._camera_points_handler)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "dynamic_step_and_hold"

        self.gait_selection.create_subscription(
            JointState,
            "/march/close/final_position",
            self._update_start_position_idle_state,
            DEFAULT_HISTORY_DEPTH,
        )
        self._final_position_pub = self.gait_selection.create_publisher(
            JointState,
            "/march/step_and_hold/final_position",
            DEFAULT_HISTORY_DEPTH,
        )

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        self._should_stop = False
        self._end = False

        self._start_time_next_command = None
        self._current_time = None

        self._next_command = None

        self._start_is_delayed = True
        self._scheduled_early = False

        if (
            self.start_position_all_joints == self.home_stand_position_all_joints and not self.start_from_left_side
        ) or (self.start_position_all_joints == END_POSITION_RIGHT):
            self.subgait_id = "right_swing"
        elif self.start_position_all_joints == END_POSITION_LEFT:
            self.subgait_id = "left_swing"
        else:
            self.subgait_id = "left_swing"

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

    def _callback_force_unknown(self, msg: GaitInstruction) -> None:
        """Reset start position to home stand after force unknown.

        Args:
            msg (GaitInstruction): message containing a gait_instruction from the IPD
        """
        if msg.type == GaitInstruction.UNKNOWN:
            self._set_start_position_to_home_stand()
            self.use_predetermined_foot_location = False
            self.subgait_id = "right_swing"
            self._trajectory_failed = False


class TrajectoryCommandHandlerStepAndHold(TrajectoryCommandHandlerFixedSizes):
    """TrajectoryCommandHandler for a step and hold."""

    _end: bool

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)

    def get_trajectory_command(
        self, subgait_id: str, start_position_all_joints: Dict[str, float], start=False, stop=False
    ) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on the _position_queue if enabled.

        Args:
            subgait_id (str): whether it is a right_swing or left_swing
            start_position_all_joints (Dict[str, float]): start joint angles of all joints
            start (bool): whether it is a start gait, default False
            stop (bool): whether it is a stop gait, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time. Returns None if the location found by
                CoViD is too old.
        """
        self.subgait_id = subgait_id
        self.start_position_all_joints = start_position_all_joints

        if stop:
            self._logger.info("Stopping dynamic gait.")
        else:
            if self._gait.use_predetermined_foot_location:
                self.foot_location = deepcopy(self._predetermined_foot_location)
                self._gait.use_predetermined_foot_location = False
            else:
                if self._use_position_queue and not self.position_queue.empty():
                    self.foot_location = self._get_foot_location_from_queue()
                elif self._use_position_queue and self.position_queue.empty():
                    self.fill_queue()
                    self._logger.warn(f"Queue is empty. Reset queue to {list(self.position_queue.queue)}.")
                    return None
                else:
                    try:
                        self.foot_location = self._points_handler.get_foot_location(self.subgait_id)
                        if self._points_handler.is_foot_location_too_old(self.foot_location):
                            return None
                    except AttributeError:
                        self._logger.info("No FootLocation found. Connect the camera or use simulated points.")
                        self._end = True
                        return None

            if not stop:
                self._points_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self._logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        return self._get_first_feasible_trajectory(start, stop)

    def _create_subgait_instance(
        self,
        start_position: Dict[str, float],
        subgait_id: str,
        start: bool,
        stop: bool,
    ) -> DynamicSubgait:
        """Create a DynamicSubgait instance.

        Args:
            start_position (Dict[str, float]): dict containing joint_names and positions of the joint as floats
            subgait_id (str): either 'left_swing' or 'right_swing'
            start (bool): whether it is a start gait or not
            stop (bool): whether it is a stop gait or not
        Returns:
            DynamicSubgait: DynamicSubgait instance for the desired step
        """
        if subgait_id == "right_swing":
            end_position = END_POSITION_RIGHT
        else:
            end_position = END_POSITION_LEFT

        # reset start_from_left_side attribute
        self._gait.start_from_left_side = False

        return DynamicSubgait(
            self._gait.gait_selection,
            end_position,
            start_position,
            subgait_id,
            self._gait.actuating_joint_names,
            self.foot_location,
            self._gait.joint_soft_limits,
            start,
            stop,
            hold_subgait=True,
        )

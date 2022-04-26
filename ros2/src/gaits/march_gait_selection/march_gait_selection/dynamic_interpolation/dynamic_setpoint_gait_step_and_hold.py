"""Author: Marten Haitjema, MVII."""

from typing import Dict
from rclpy.node import Node

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_goniometric_ik_solver.ik_solver import Pose
from march_utility.utilities.logger import Logger

from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import Point

HOLD_FOOT_POSITION = FootPosition(duration=1.3, processed_point=Point(x=0.0, y=0.07, z=0.45))


class DynamicSetpointGaitStepAndHold(DynamicSetpointGaitStepAndClose):
    """Class for stepping to a hold position firstly, and to the desired position secondly."""

    def __init__(self, gait_selection_node: Node):
        self.subgait_id = "right_swing"
        self._end_position_right = {}
        self._end_position_left = {}
        super().__init__(gait_selection_node)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "dynamic_step_and_hold"

        # TODO: remove local all_joint_names variable after 'close gait for step' is merged
        all_joint_names = list(self.home_stand_position_all_joints.keys())
        pose_right = Pose(all_joint_names).solve_end_position(
            HOLD_FOOT_POSITION.processed_point.x,
            HOLD_FOOT_POSITION.processed_point.y,
            HOLD_FOOT_POSITION.processed_point.z,
            "right_swing",
        )
        pose_left = Pose(all_joint_names).solve_end_position(
            HOLD_FOOT_POSITION.processed_point.x,
            HOLD_FOOT_POSITION.processed_point.y,
            HOLD_FOOT_POSITION.processed_point.z,
            "left_swing",
        )

        for i, name in enumerate(all_joint_names):
            self._end_position_right[name] = pose_right[i]
            self._end_position_left[name] = pose_left[i]

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        self._should_stop = False
        self._end = False

        self._start_time_next_command = None
        self._current_time = None

        self._next_command = None

        self._start_is_delayed = True
        self._scheduled_early = False

        if self.start_position_all_joints == self._end_position_right:
            self.subgait_id = "right_swing"
        elif self.start_position_all_joints == self._end_position_left:
            self.subgait_id = "left_swing"

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
        if self.subgait_id == "right_swing":
            return DynamicSubgait(
                self.gait_selection,
                self._end_position_right,
                start_position,
                subgait_id,
                self.joint_names,
                self.foot_location,
                self.joint_soft_limits,
                start,
                stop,
            )
        else:
            return DynamicSubgait(
                self.gait_selection,
                self._end_position_left,
                start_position,
                subgait_id,
                self.joint_names,
                self.foot_location,
                self.joint_soft_limits,
                start,
                stop,
            )

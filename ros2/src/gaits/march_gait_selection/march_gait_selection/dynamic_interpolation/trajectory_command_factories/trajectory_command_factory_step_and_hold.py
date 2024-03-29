"""Author: Marten Haitjema, MVII."""

from typing import Dict, Optional
from copy import deepcopy

from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory_fixed_sizes import (
    TrajectoryCommandFactoryFixedSizes,
)
from march_gait_selection.dynamic_interpolation.gaits.dynamic_step import DynamicStep
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    STEPPING_STONES_END_POSITION_RIGHT,
    STEPPING_STONES_END_POSITION_LEFT,
)


class TrajectoryCommandFactoryStepAndHold(TrajectoryCommandFactoryFixedSizes):
    """TrajectoryCommandFactory for a step and hold."""

    _end: bool

    def __init__(self, gait, point_handler):
        super().__init__(gait, point_handler)
        self._logger = gait.node.get_logger().get_child(__class__.__name__)

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
                        self.foot_location = self._point_handler.get_foot_location(self.subgait_id)
                        if self._point_handler.is_foot_location_too_old(self.foot_location):
                            return None
                    except AttributeError:
                        self._logger.info("No FootLocation found. Connect the camera or use simulated points.")
                        self._end = True
                        return None

            if not stop:
                self._point_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self._logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        return self._create_and_validate_trajectory_command(start, stop)

    def _create_trajectory_command(
        self,
        start_position: Dict[str, float],
        subgait_id: str,
        start: bool,
        stop: bool,
    ) -> TrajectoryCommand:
        """Create a DynamicStep instance and return the joint_trajectory_msg.

        Args:
            start_position (Dict[str, float]): dict containing joint_names and positions of the joint as floats
            subgait_id (str): either 'left_swing' or 'right_swing'
            start (bool): whether it is a start gait or not
            stop (bool): whether it is a stop gait or not
        Returns:
            TrajectoryCommand: a trajectory command
        """
        if subgait_id == "right_swing":
            end_position = STEPPING_STONES_END_POSITION_RIGHT
        else:
            end_position = STEPPING_STONES_END_POSITION_LEFT

        # reset start_from_left_side attribute
        self._gait.start_from_left_side = False

        self.dynamic_step = DynamicStep(
            self._gait.node,
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

        return TrajectoryCommand(
            self.dynamic_step.get_joint_trajectory_msg(self._gait.add_push_off),
            Duration(self.foot_location.duration),
            subgait_id,
            self._gait.start_time_next_command,
        )

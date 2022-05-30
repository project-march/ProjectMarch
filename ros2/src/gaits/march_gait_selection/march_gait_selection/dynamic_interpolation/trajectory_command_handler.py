"""Author: Marten Haitjema, MVII."""

from math import floor
from typing import Optional, Dict

from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_shared_msgs.msg import FootPosition
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError, VelocitySoftLimitError
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger

DURATION_INCREASE_FACTOR = 1.5
DURATION_INCREASE_SIZE = 0.25


class TrajectoryCommandHandler:
    """Class that creates and validates a trajectory command."""

    subgait_id: str
    foot_location: FootPosition
    start_position_all_joints: Dict[str, float]

    def __init__(self, gait, points_handler):
        self._gait = gait
        self._points_handler = points_handler
        self._logger = Logger(self._gait.gait_selection, __class__.__name__)
        self._trajectory_failed = False

    def get_trajectory_command(
        self, subgait_id: str, start_position_all_joints: Dict[str, float], start=False, stop=False
    ) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id.

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
            self._gait._end = True
            self._logger.info("Stopping dynamic gait.")
        else:
            try:
                self.foot_location = self._points_handler.get_foot_location(self.subgait_id)
                stop = self._points_handler.is_foot_location_too_old(self.foot_location)
            except AttributeError:
                self._logger.warn("No FootLocation found. Connect the camera or use simulated points.")
                self._gait._end = True
                return None
            if not stop:
                self._points_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self._logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        if start and stop:
            # If it is a start gait and stop is set to true because of the message time, do not return a trajectory.
            return None

        return self._get_first_feasible_trajectory(start, stop)

    def _get_first_feasible_trajectory(self, start: bool, stop: bool) -> Optional[TrajectoryCommand]:
        """Returns the first trajectory than can be executed.

        If a subgait is not feasible, it will first try to increase the duration. If it is
        still not feasible, execution of the gait will be stopped.

        Args:
            start (:obj: bool, optional): whether` it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        original_duration = self.foot_location.duration
        max_iteration = (
            floor(((original_duration * DURATION_INCREASE_FACTOR - original_duration) / DURATION_INCREASE_SIZE)) - 1
        )
        iteration = 0

        while not self._is_duration_bigger_than_max_duration(original_duration):
            is_final_iteration = iteration == max_iteration
            trajectory_command = self._try_to_get_trajectory_command(
                start,
                stop,
                original_duration,
                iteration,
                is_final_iteration,
            )
            # Return command if current and next step can be made at same duration
            second_step = self._can_get_second_step(is_final_iteration)
            if trajectory_command is not None and second_step:
                self._trajectory_failed = False
                self._gait.update_start_position_gait_state()
                return trajectory_command
            else:
                self._trajectory_failed = True
                self.foot_location.duration += DURATION_INCREASE_SIZE
            iteration += 1

        # If no feasible subgait can be found, try to execute close gait
        if not start:
            try:
                return self._get_stop_gait()
            except (PositionSoftLimitError, VelocitySoftLimitError, ValueError) as e:
                self._logger.warn(f"Can not get stop gait. {e.msg}")

        # If close gait is not feasible, stop gait completely
        self._gait._end = True
        return None

    def _try_to_get_trajectory_command(
        self,
        start: bool,
        stop: bool,
        original_duration: float,
        iteration: int,
        is_final_iteration: bool,
    ) -> Optional[TrajectoryCommand]:
        """Try to get a joint_trajectory_msg from the dynamic subgait instance.

        Args:
            start (bool): whether it is a start gait
            stop (bool): whether it is a stop gait
            original_duration (float): original duration of the gait as set in the GaitPreprocessor
            iteration (int): current iteration over the velocity
            is_final_iteration (bool): True if current iteration equals the maximum amount of iterations
        Returns:
            TrajectoryCommand: optional command if successful, otherwise None
        """
        try:
            self.dynamic_subgait = self._create_subgait_instance(
                self.start_position_all_joints, self.subgait_id, start, stop
            )
            trajectory = self.dynamic_subgait.get_joint_trajectory_msg(self._gait.add_push_off)
            self._logger.debug(
                f"Found trajectory after {iteration + 1} iterations at duration of {self.foot_location.duration}. "
                f"Original duration was {original_duration}."
            )
            return TrajectoryCommand(
                trajectory,
                Duration(self.foot_location.duration),
                self.subgait_id,
                self._gait.start_time_next_command,
            )
        except (PositionSoftLimitError, VelocitySoftLimitError, ValueError) as e:
            if is_final_iteration:
                self._logger.warn(
                    f"Can not get trajectory after {iteration + 1} iterations. {e.msg} Gait will not be executed."
                )
            return None

    def _can_get_second_step(self, is_final_iteration: bool) -> bool:
        """Tries to create the subgait that is one step ahead.

        If this is not possible, the first subgait should not be executed.

        Args:
            is_final_iteration (bool): True if current iteration equals the maximum amount of iterations
        Returns:
            bool: True if second step can be made, otherwise false
        """
        start_position = self.dynamic_subgait.get_final_position()
        subgait_id = "right_swing" if self.subgait_id == "left_swing" else "left_swing"
        subgait = self._create_subgait_instance(
            start_position,
            subgait_id,
            start=False,
            stop=False,
        )
        try:
            subgait.get_joint_trajectory_msg(self._gait.add_push_off)
        except (PositionSoftLimitError, VelocitySoftLimitError, ValueError) as e:
            if is_final_iteration:
                self._logger.warn(f"Second step is not feasible. {e.msg}")
            return False
        return True

    def _get_stop_gait(self) -> Optional[TrajectoryCommand]:
        """Returns a TrajectoryCommand containing a stop gait.

        Returns:
            TrajectoryCommand: command containing a stop gait
        """
        self._gait._end = True
        subgait = self._create_subgait_instance(
            self.start_position_all_joints,
            self.subgait_id,
            start=False,
            stop=True,
        )
        trajectory = subgait.get_joint_trajectory_msg(self._gait.add_push_off)
        return TrajectoryCommand(
            trajectory,
            Duration(self.foot_location.duration),
            self.subgait_id,
            self._gait.start_time_next_command,
        )

    def _is_duration_bigger_than_max_duration(self, original_duration: float) -> bool:
        """Returns true if duration is bigger than maximum duration, else false.

        Args:
            original_duration (float): duration before iterations
        Returns:
            bool: True if current duration is bigger than max allowed duration, else False
        """
        return self.foot_location.duration >= original_duration * DURATION_INCREASE_FACTOR

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
        return DynamicSubgait(
            self._gait.gait_selection,
            self._gait.home_stand_position_all_joints,
            start_position,
            subgait_id,
            self._gait.actuating_joint_names,
            self.foot_location,
            self._gait.joint_soft_limits,
            start,
            stop,
        )

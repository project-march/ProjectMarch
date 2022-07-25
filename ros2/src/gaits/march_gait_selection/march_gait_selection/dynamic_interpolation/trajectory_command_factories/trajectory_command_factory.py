"""Author: Marten Haitjema, MVII."""

from typing import Optional, Dict

from march_gait_selection.dynamic_interpolation.gaits.dynamic_step import DynamicStep
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError, VelocitySoftLimitError, GaitError
from march_utility.utilities.duration import Duration
from march_shared_msgs.msg import FootPosition

DURATION_INCREASE_FACTOR = 1.5
DURATION_INCREASE_SIZE = 0.25


class TrajectoryCommandFactory:
    """Class that creates and validates a trajectory command.

    Args:
        gait: The gait class
        point_handler: The points handler class

    Attributes:
        subgait_id (str): either 'left_swing' or 'right_swing'
        start_position_all_joints (Dict[str, float]): start joint angles of all joints
        foot_location (FootPosition): foot position message to step towards
        dynamic_step (DynamicStep): instance of the DynamicStep class, used to get trajectory msg

        _gait: the gait class
        _point_handler: the points handler class
        _logger (Logger): used to log with the class name as a prefix
    """

    subgait_id: str
    foot_location: FootPosition
    start_position_all_joints: Dict[str, float]

    def __init__(self, gait, point_handler):
        self._gait = gait
        self._point_handler = point_handler
        self._logger = gait.node.get_logger().get_child(__class__.__name__)

    @property
    def final_position(self) -> Dict[str, float]:
        """Returns the position that the gait ends in."""
        return self._final_position

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
                self.foot_location = self._point_handler.get_foot_location(self.subgait_id)
                stop = self._point_handler.is_foot_location_too_old(self.foot_location)
            except AttributeError:
                self._logger.error("No FootLocation found. Connect the camera or use a gait with a fixed step size.")
                self._gait._end = True
                return None if start else self._get_stop_gait()
            if not stop:
                self._point_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self._logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        if start and stop:
            # If it is a start gait and stop is set to true because of the message time, do not return a trajectory.
            return None

        return self._create_and_validate_trajectory_command(start, stop)

    def _create_and_validate_trajectory_command(self, start: bool, stop: bool) -> Optional[TrajectoryCommand]:
        """Creates and returns a TrajectoryCommand if first and second step are feasible.

        If first or second step is not feasible, tries to get a close gait. If this is also not possible, returns None.

        Arguments:
            start (bool): whether it is a start gait.
            stop (bool): whether it is a stop gait.

        Returns:
            Optional[TrajectoryCommand]: TrajectoryCommand if feasible, else None.
        """
        try:
            trajectory_command = self._create_trajectory_command(
                self.start_position_all_joints,
                self.subgait_id,
                start,
                stop,
            )
            self._final_position = self.dynamic_step.get_final_position()
            self._is_second_step_possible = self._can_get_second_step()
            self._gait.update_start_position_gait_state()
            return trajectory_command  # noqa R504 variable assignment is necessary to get dynamic_step instance
        except (PositionSoftLimitError, VelocitySoftLimitError, ValueError) as e:
            if not start:
                self._logger.error(f"{self._get_error_msg(e)}")
                return self._get_stop_gait()
            else:
                raise GaitError(self._get_error_msg(e))

    def _get_error_msg(self, error) -> str:
        if self._is_second_step_possible:
            return f"Step is not possible. {error}"
        else:
            return f"Second step is not possible {error}"

    def _can_get_second_step(self) -> bool:
        """Tries to create the subgait that is one step ahead.

        If this is not possible, the first subgait should not be executed.

        Returns:
            bool: True if second step can be made, otherwise false.
        """
        start_position = self.dynamic_step.get_final_position()
        subgait_id = "right_swing" if self.subgait_id == "left_swing" else "left_swing"
        return self._create_trajectory_command(start_position, subgait_id, start=False, stop=False) is not None

    def _get_stop_gait(self) -> TrajectoryCommand:
        """Returns a TrajectoryCommand containing a stop gait.

        Returns:
            TrajectoryCommand: command containing a stop gait
        """
        self._gait._end = True
        try:
            return self._create_trajectory_command(
                self.start_position_all_joints, self.subgait_id, start=False, stop=True
            )
        except (PositionSoftLimitError, VelocitySoftLimitError, ValueError) as e:
            raise GaitError(f"Can not get stop gait. {e}")

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
        self.dynamic_step = DynamicStep(
            self._gait.node,
            self._gait.home_stand_position_all_joints,
            start_position,
            subgait_id,
            self._gait.actuating_joint_names,
            self.foot_location,
            self._gait.joint_soft_limits,
            start,
            stop,
        )

        return TrajectoryCommand(
            self.dynamic_step.get_joint_trajectory_msg(self._gait.add_push_off),
            Duration(self.foot_location.duration),
            self.subgait_id,
            self._gait.start_time_next_command,
        )

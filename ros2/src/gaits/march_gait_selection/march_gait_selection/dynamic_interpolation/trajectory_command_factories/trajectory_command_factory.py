"""Author: Marten Haitjema, MVII."""

import os
import yaml

from typing import Optional, Dict, Tuple
from queue import Queue
from ament_index_python.packages import get_package_share_path

from march_gait_selection.dynamic_interpolation.gaits.dynamic_step import DynamicStep
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError, VelocitySoftLimitError, GaitError
from march_utility.utilities.duration import Duration
from march_shared_msgs.msg import FootPosition

from std_msgs.msg import Header
from geometry_msgs.msg import Point

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
        position_queue (List[Dict[str, float]]): List containing foot position dictionaries for x, y and z coordinates.
            Defined in _position_queue.yaml
        duration_from_yaml (float): duration of the step as specified in _position_queue.yaml

        _use_position_queue (bool): True if _position_queue will be used instead of covid points, else False
        _gait: the gait class
        _point_handler: the points handler class
        _logger (Logger): used to log with the class name as a prefix
    """

    subgait_id: str
    foot_location: FootPosition
    start_position_all_joints: Dict[str, float]
    _stop: bool
    _use_position_queue: bool

    def __init__(self, gait, point_handler):
        self._gait = gait
        self._point_handler = point_handler
        self._logger = gait.node.get_logger().get_child(__class__.__name__)
        self._create_position_queue()
        self.update_parameter()
        self._is_second_step_possible = False

    @property
    def final_position(self) -> Dict[str, float]:
        """Returns the position that the gait ends in."""
        return self._final_position

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
        self._stop = stop
        self.subgait_id = subgait_id
        self.start_position_all_joints = start_position_all_joints

        if self._stop:
            self.foot_location.processed_point.x = 0.0
            self.foot_location.processed_point.y = 0.0
            self.foot_location.processed_point.z = 0.51
            self._gait._end = True
            self._logger.info("Stopping dynamic gait.")
        else:
            if self._use_position_queue:
                self.foot_location = self._check_if_queue_is_not_empty_and_get_foot_location()
            else:
                self.foot_location = self._get_foot_location_from_point_handler()

        if self.foot_location is None:
            return None if start else self._get_stop_gait()

        self._point_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
        self._logger.info(
            f"Stepping to location ({self.foot_location.processed_point.x}, "
            f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
        )

        return self._create_and_validate_trajectory_command(start, self._stop)

    def _create_and_validate_trajectory_command(self, start: bool, stop: bool) -> Optional[TrajectoryCommand]:
        """Creates and returns a TrajectoryCommand if first and second step are feasible.

        If first or second step is not feasible, tries to get a close gait. If this is also not possible, returns None.

        Arguments:
            start (bool): whether it is a start gait.
            stop (bool): whether it is a stop gait.

        Returns:
            Optional[TrajectoryCommand]: TrajectoryCommand if feasible, else None.
        """
        trajectory_command, success = self._get_first_step(start, stop)
        is_second_step_possible = self._can_get_second_step() if success else True

        return trajectory_command if is_second_step_possible else (self._get_stop_gait() if not start else None)

    def _get_first_step(self, start: bool, stop: bool) -> Tuple[Optional[TrajectoryCommand], bool]:
        """Tries to create the subgait for the given foot position.

        Arguments:
            start (bool): whether it is a start gait.
            stop (bool): whether it is a stop gait.

        Returns:
            Optional[TrajectoryCommand]: TrajectoryCommand if feasible, else None.
            bool: whether the step has been created successfully.

        Raises:
            GaitError: when the first step is not possible, and it is a start gait.
        """
        try:
            trajectory_command = self._create_trajectory_command(
                self.start_position_all_joints,
                self.subgait_id,
                start,
                stop,
            )
            self._final_position = self.dynamic_step.get_final_position()
            self._gait.update_start_position_gait_state()
            return trajectory_command, True
        except (PositionSoftLimitError, VelocitySoftLimitError, ValueError) as e:
            if not start:
                self._logger.error(f"Step is not possible. {e}")
                return self._get_stop_gait(), False
            else:
                raise GaitError(f"First step is not possible. {e}")

    def _can_get_second_step(self) -> bool:
        """Tries to create the subgait that is one step ahead.

        If this is not possible, the first subgait should not be executed.

        Returns:
            bool: True if second step can be made, otherwise false.
        """
        start_position = self.dynamic_step.get_final_position()
        subgait_id = "right_swing" if self.subgait_id == "left_swing" else "left_swing"
        try:
            self._create_trajectory_command(start_position, subgait_id, start=False, stop=False)
        except (PositionSoftLimitError, VelocitySoftLimitError, ValueError) as e:
            self._logger.error(f"Second step is not possible. {e}")
            return False
        return True

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

    def _check_if_queue_is_not_empty_and_get_foot_location(self) -> Optional[FootPosition]:
        """If the queue is empty, the gait is closed and the queue is reset. Else, gets location from queue."""
        if self.position_queue.empty():
            self._close_gait_and_reset_queue()
            return self.foot_location
        else:
            return self._get_foot_location_from_queue()

    def _get_foot_location_from_point_handler(self) -> Optional[FootPosition]:
        """Returns a FootPosition message from the point handler, if available."""
        try:
            self.foot_location = self._point_handler.get_foot_location(self.subgait_id)
            msg_too_old, msg = self._point_handler.is_foot_location_too_old(self.foot_location)
            if msg_too_old:
                self._gait._end = True
                self._stop = True
                self._logger.error(msg)
            return self.foot_location
        except AttributeError:
            self._logger.warn("No FootLocation found. Connect the camera or use a gait with a fixed step size.")
            self._gait._end = True
            return None

    def _close_gait_and_reset_queue(self) -> None:
        """Closes the gait and reset the queue after the queue is empty."""
        self.fill_queue()
        self._logger.warn(f"Queue is empty. Closing the gait. Resetting queue to {list(self.position_queue.queue)}")
        self._stop = True
        self._gait._end = True

    def _get_foot_location_from_queue(self) -> FootPosition:
        """Get FootPosition message from the position queue.

        Returns:
            FootPosition: FootPosition msg with position from queue
        """
        header = Header(stamp=self._gait.node.get_clock().now().to_msg())
        point_from_queue = self.position_queue.get()
        point = Point(x=point_from_queue["x"], y=point_from_queue["y"], z=point_from_queue["z"])

        return FootPosition(header=header, processed_point=point, duration=self.duration_from_yaml)

    def update_parameter(self) -> None:
        """Updates '_use_position_queue' to the newest value in gait_node."""
        self._use_position_queue = self._gait.node.use_position_queue

    def _create_position_queue(self) -> None:
        """Creates and fills the queue with values from position_queue.yaml."""
        queue_path = get_package_share_path("march_gait_selection")
        queue_file_loc = os.path.join(queue_path, "position_queue", "position_queue.yaml")
        try:
            with open(queue_file_loc, "r") as queue_file:
                position_queue_yaml = yaml.load(queue_file, Loader=yaml.SafeLoader)
        except OSError as e:
            self._logger.error(f"Position queue file does not exist. {e}")

        self.duration_from_yaml = position_queue_yaml["duration"]
        self.points_from_yaml = position_queue_yaml["points"]
        self.position_queue = Queue()
        self.fill_queue()

    def fill_queue(self) -> None:
        """Fills the position queue with the values specified in position_queue.yaml."""
        for point in self.points_from_yaml:
            self.position_queue.put(point)

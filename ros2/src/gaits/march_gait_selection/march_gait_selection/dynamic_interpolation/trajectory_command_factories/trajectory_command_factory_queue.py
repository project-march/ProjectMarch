"""Author: Marten Haitjema, MVII."""

import os
import yaml

from queue import Queue
from typing import Dict, Optional
from ament_index_python.packages import get_package_share_path

from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH


class TrajectoryCommandFactoryQueue(TrajectoryCommandFactory):
    """Class that creates and validates a trajectory command and has the ability to put position in a queue.

    Attributes:
        position_queue (List[Dict[str, float]]): List containing foot position dictionaries for x, y and z coordinates.
            Defined in _position_queue.yaml
        duration_from_yaml (float): duration of the step as specified in _position_queue.yaml
        _use_position_queue (bool): True if _position_queue will be used instead of covid points, else False
    """

    subgait_id: str
    start_position_all_joints: Dict[str, float]
    foot_location: FootPosition
    _use_position_queue: bool
    _end: bool

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)
        self._logger = gait.node.get_logger().get_child(__class__.__name__)
        self._create_position_queue()
        self.update_parameter()
        self._trajectory_failed = False

        self._gait.node.create_subscription(
            Point,
            "/march/step/add_point_to_queue",
            self._add_point_to_queue,
            DEFAULT_HISTORY_DEPTH,
        )

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

        if self._use_position_queue and not self.position_queue.empty():
            self.foot_location = self._get_foot_location_from_queue()
        elif self._use_position_queue and self.position_queue.empty():
            self.fill_queue()
            self._logger.info(f"Queue is empty. Resetting queue to {list(self.position_queue.queue)}")
            stop = True
            self._end = True
        else:
            try:
                self.foot_location = self._points_handler.get_foot_location(self.subgait_id)
                stop = self._points_handler.is_foot_location_too_old(self.foot_location)
                self._points_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
            except AttributeError:
                self._logger.warn("No FootLocation found. Connect the camera or use a gait with a fixed step size.")
                self._end = True
                return None

        self._logger.info(
            f"Stepping to location ({self.foot_location.processed_point.x}, "
            f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
        )

        return self._get_first_feasible_trajectory(start, stop)

    def _get_foot_location_from_queue(self) -> FootPosition:
        """Get FootPosition message from the position queue.

        Returns:
            FootPosition: FootPosition msg with position from queue
        """
        header = Header(stamp=self._gait.node.get_clock().now().to_msg())
        point_from_queue = self.position_queue.get()
        point = Point(x=point_from_queue["x"], y=point_from_queue["y"], z=point_from_queue["z"])

        if self.position_queue.empty():
            self._logger.warn("Next step will be a close gait.")

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

    def _add_point_to_queue(self, point: Point) -> None:
        """Adds a point to the end of the queue.

        Args:
            point (Point): point message to add to the queue.
        """
        point_dict = {"x": point.x, "y": point.y, "z": point.z}
        self.position_queue.put(point_dict)
        self._logger.info(f"Point added to position queue. Current queue is: {list(self.position_queue.queue)}")

    def _can_get_second_step(self, final_iteration: bool) -> bool:
        """Returns true if second step is possible, always true for single step."""
        return True

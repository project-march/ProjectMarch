"""Author: Marten Haitjema, MVII."""

import os
import yaml

from queue import Queue
from rclpy.node import Node
from rclpy.time import Time
from typing import Optional, Dict
from ament_index_python.packages import get_package_share_path

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from sensor_msgs.msg import JointState

from march_utility.gait.edge_position import EdgePosition, UnknownEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition


class DynamicSetpointGaitStep(DynamicSetpointGait):
    """Step gait based on dynamic setpoint gait.

    Args:
        gait_selection_node (GaitSelection): the gait selection node

    Attributes:
        position_queue (List[Dict[str, float]]): List containing foot position dictionaries for x, y and z coordinates.
            Defined in _position_queue.yaml
        duration_from_yaml (float): duration of the step as specified in _position_queue.yaml
        _use_position_queue (bool): True if _position_queue will be used instead of covid points, else False
    """

    _current_time: Optional[Time]
    _use_position_queue: bool

    def __init__(self, node: Node, positions: Dict[str, EdgePosition]):
        super().__init__(node, positions)
        self._logger = Logger(self._node, __class__.__name__)
        self.subgait_id = "right_swing"
        self.gait_name = "dynamic_step"
        self.update_parameters()
        self._create_position_queue()

        self._node.create_subscription(
            Point,
            "/march/step/add_point_to_queue",
            self._add_point_to_queue,
            DEFAULT_HISTORY_DEPTH,
        )
        self.final_position_pub = self._node.create_publisher(
            JointState,
            "/march/gait_selection/final_position",
            DEFAULT_HISTORY_DEPTH,
        )

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        if self.start_position_actuating_joints == self.home_stand_position_actuating_joints:
            self.subgait_id = "right_swing"

        self._should_stop = False
        self._end = False

        self._start_time_next_command = None
        self._current_time = None

        self._next_command = None

        self._start_is_delayed = True
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
        if self._start_is_delayed:
            if current_time >= self._start_time_next_command:
                return self._update_start_subgait()
            else:
                return GaitUpdate.empty()

        if current_time >= self._start_time_next_command:
            return self._update_state_machine()

        return GaitUpdate.empty()

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the single single step has finished. Also switches the subgait_id.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        if not self._trajectory_failed:
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"

        if self._end:
            self.subgait_id = "right_swing"
            if not isinstance(self.final_position, UnknownEdgePosition):
                self.final_position_pub.publish(JointState(position=self.final_position.values))

        return GaitUpdate.finished()

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on the _position_queue if enabled.

        Args:
            start (:obj: bool, optional): whether it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time. Returns None if the location found by
                CoViD is too old.
        """
        if self._use_position_queue and not self.position_queue.empty():
            self.foot_location = self._get_foot_location_from_queue()
        elif self._use_position_queue and self.position_queue.empty():
            self._fill_queue()
            self._logger.info(f"Queue is empty. Resetting queue to {list(self.position_queue.queue)}")
            stop = True
            self._end = True
        else:
            try:
                self.foot_location = self._get_foot_location(self.subgait_id)
                stop = self._check_msg_time(self.foot_location)
                self._publish_chosen_foot_position(self.subgait_id, self.foot_location)
            except AttributeError:
                self._logger.info("No FootLocation found. Connect the camera or use simulated points.")
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
        header = Header(stamp=self._node.get_clock().now().to_msg())
        point_from_queue = self.position_queue.get()
        point = Point(x=point_from_queue["x"], y=point_from_queue["y"], z=point_from_queue["z"])

        if self.position_queue.empty():
            self._logger.warn("Next step will be a close gait.")

        return FootPosition(header=header, processed_point=point, duration=self.duration_from_yaml)

    def update_parameters(self) -> None:
        """Updates '_use_position_queue' to the newest value in gait_node."""
        super().update_parameters()
        self._use_position_queue = self._node.use_position_queue

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
        self._fill_queue()

    def _fill_queue(self) -> None:
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

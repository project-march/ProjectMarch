"""Author: Marten Haitjema, MVII."""

import os
import yaml

from copy import copy
from queue import Queue
from typing import Dict, Optional
from ament_index_python import get_package_share_path
from rclpy.node import Node

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from sensor_msgs.msg import JointState

from march_utility.gait.edge_position import EdgePosition, UnknownEdgePosition
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger
from march_utility.exceptions.gait_exceptions import WrongStartPositionError

from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String


class DynamicSetpointGaitStepAndHold(DynamicSetpointGaitStepAndClose):
    """Class for stepping to a hold position firstly, and to the desired position secondly."""

    _use_position_queue: bool

    def __init__(self, node: Node, positions: Dict[str, EdgePosition]):
        self.subgait_id = "right_swing"
        self._end_position_right = {}
        self._end_position_left = {}
        super().__init__(node, positions)
        self.logger = Logger(self._node, __class__.__name__)
        self.gait_name = "dynamic_step_and_hold"

        self.update_parameters()
        self._create_position_queue()

        self._node.create_subscription(
            Point,
            "/march/step_and_hold/add_point_to_queue",
            self._add_point_to_queue,
            DEFAULT_HISTORY_DEPTH,
        )
        self._node.create_subscription(
            String,
            "/march/step_and_hold/start_side",
            self._set_start_subgait_id,
            DEFAULT_HISTORY_DEPTH,
        )
        self.final_position_pub = self._node.create_publisher(
            JointState,
            "/march/gait_selection/final_position",
            DEFAULT_HISTORY_DEPTH,
        )

        self._end_position_right = copy(self.home_stand_position_all_joints)
        self._end_position_right["right_hip_aa"] = 0
        self._end_position_right["left_hip_aa"] = 0
        self._end_position_right["right_hip_fe"] = 0
        self._end_position_right["left_hip_fe"] = 0
        self._end_position_right["right_knee"] = 1

        self._end_position_left = copy(self.home_stand_position_all_joints)
        self._end_position_left["right_hip_aa"] = 0
        self._end_position_left["left_hip_aa"] = 0
        self._end_position_left["right_hip_fe"] = 0
        self._end_position_left["left_hip_fe"] = 0
        self._end_position_left["left_knee"] = 1

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

    def _set_and_get_next_command(self) -> Optional[TrajectoryCommand]:
        """Create the next command. Because it is a single step, this will always be a left_swing and a close gait.

        Returns:
            TrajectoryCommand: A TrajectoryCommand for the next subgait
        """
        if not self._trajectory_failed:
            if self.subgait_id == "right_swing":
                self.subgait_id = "left_swing"
            elif self.subgait_id == "left_swing":
                self.subgait_id = "right_swing"

        if self._end:
            if not isinstance(self.final_position, UnknownEdgePosition):
                self.final_position_pub.publish(JointState(position=self.final_position.values))
            # If the gait has ended, the next command should be None
            return None
        else:
            self._end = True
            return self._get_trajectory_command(stop=True)

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
            end_position = self._end_position_right
        else:
            end_position = self._end_position_left

        return DynamicSubgait(
            self._node,
            end_position,
            start_position,
            subgait_id,
            self.actuating_joint_names,
            self.foot_location,
            self.joint_soft_limits,
            start,
            stop,
            hold_subgait=True,
        )

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id, or based on the _position_queue if enabled.

        Args:
            start (Optional[bool]): whether it is a start gait or not, default False
            stop (Optional[bool]): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time. Returns None if the position queue is
                empty, if the location found by CoViD is too old, or if CoViD has not found any location.
        """
        if stop:
            self.logger.info("Stopping dynamic gait.")
        else:
            if self._use_position_queue and not self.position_queue.empty():
                self.foot_location = self._get_foot_location_from_queue()
            elif self._use_position_queue and self.position_queue.empty():
                self.logger.warn(f"Queue is empty. Resetting queue to {list(self.position_queue.queue)}.")
                self._fill_queue()
                return None
            else:
                try:
                    self.foot_location = self._get_foot_location(self.subgait_id)
                    if self._check_msg_time(self.foot_location):
                        return None
                except AttributeError:
                    self.logger.info("No FootLocation found. Connect the camera or use simulated points.")
                    self._end = True
                    return None

        if not stop:
            self._publish_chosen_foot_position(self.subgait_id, self.foot_location)
            self.logger.info(
                f"Stepping to location ({self.foot_location.processed_point.x}, "
                f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
            )

        return self._get_first_feasible_trajectory(start, stop)

    def _can_get_second_step(self, is_final_iteration: bool) -> bool:
        """Tries to create the subgait that is one step ahead, which is a stop gait for step and close.

        This safety check is not relevant for step and hold and thus always returns True.
        """
        return True

    def _get_foot_location_from_queue(self) -> FootPosition:
        """Get FootPosition message from the position queue.

        Returns:
            FootPosition: FootPosition msg with position from queue
        """
        header = Header(stamp=self._node.get_clock().now().to_msg())
        point_from_queue = self.position_queue.get()
        point = Point(x=point_from_queue["x"], y=point_from_queue["y"], z=point_from_queue["z"])

        if self.position_queue.empty():
            self.logger.warn("Position queue empty. Use force unknown + homestand to close the gait.")

        return FootPosition(header=header, processed_point=point, duration=self.duration_from_yaml)

    def update_parameters(self) -> None:
        """Updates '_use_position_queue' to the newest value in the gait_node."""
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
            self.logger.error(f"Position queue file does not exist. {e}")

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
        self.logger.info(f"Point added to position queue. Current queue is: {list(self.position_queue.queue)}")

    def _set_start_subgait_id(self, start_side: String) -> None:
        """Sets the subgait_id to the given start side, if and only if exo is in homestand."""
        try:
            if self.start_position_all_joints == self.home_stand_position_all_joints:
                self.subgait_id = start_side.data
                self.logger.info(f"Starting subgait set to {self.subgait_id}")
            else:
                raise WrongStartPositionError(self.home_stand_position_all_joints, self.start_position_all_joints)
        except WrongStartPositionError as e:
            self.logger.warn(f"Can only change start side in home stand position. {e.msg}")

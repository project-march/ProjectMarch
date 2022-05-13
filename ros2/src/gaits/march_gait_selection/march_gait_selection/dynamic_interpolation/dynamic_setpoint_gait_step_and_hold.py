"""Author: Marten Haitjema, MVII."""

import os
import yaml

from copy import copy, deepcopy
from queue import Queue
from typing import Dict, Optional
from ament_index_python import get_package_share_path
from rclpy.node import Node
from sensor_msgs.msg import JointState

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger
from march_utility.exceptions.gait_exceptions import WrongStartPositionError
from march_utility.utilities.utility_functions import get_position_from_yaml

from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String

PREDETERMINED_FOOT_LOCATIONS = {
    "small_narrow": FootPosition(duration=1.5, processed_point=Point(x=0.45, y=0.0, z=0.44699999999999995)),
    "small_wide": FootPosition(duration=1.5, processed_point=Point(x=0.55, y=0.0, z=0.44699999999999995)),
    "large_narrow": FootPosition(duration=1.5, processed_point=Point(x=0.65, y=0.0, z=0.44699999999999995)),
    "large_wide": FootPosition(duration=1.5, processed_point=Point(x=0.75, y=0.0, z=0.44699999999999995)),
}

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
        self._use_predetermined_foot_location = False
        self._start_from_left_side = False
        super().__init__(gait_selection_node)
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self.gait_name = "dynamic_step_and_hold"

        self.update_parameter()
        self._create_position_queue()

        self.gait_selection.create_subscription(
            Point,
            "/march/step_and_hold/add_point_to_queue",
            self._add_point_to_queue,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            String,
            "/march/step_and_hold/start_side",
            self._set_start_subgait_id,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            String,
            "/march/step_and_hold/step_size",
            self._predetermined_foot_location_callback,
            DEFAULT_HISTORY_DEPTH,
        )
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
            self.start_position_all_joints == self.home_stand_position_all_joints and not self._start_from_left_side
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
        self._final_position_pub.publish(JointState(position=self.dynamic_subgait.get_final_position().values()))
        if self._next_command is None:
            return GaitUpdate.finished()

        self._update_time_stamps(self._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()

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

        # reset _start_from_left_side attribute
        self._start_from_left_side = False

        return DynamicSubgait(
            self.gait_selection,
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
            if self._use_predetermined_foot_location:
                self.foot_location = deepcopy(self._predetermined_foot_location)
                self._use_predetermined_foot_location = False
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
                        if self._is_foot_location_too_old(self.foot_location):
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
        header = Header(stamp=self.gait_selection.get_clock().now().to_msg())
        point_from_queue = self.position_queue.get()
        point = Point(x=point_from_queue["x"], y=point_from_queue["y"], z=point_from_queue["z"])

        if self.position_queue.empty():
            self.logger.warn("Position queue empty. Use force unknown + homestand to close the gait.")

        return FootPosition(header=header, processed_point=point, duration=self.duration_from_yaml)

    def update_parameter(self) -> None:
        """Updates '_use_position_queue' to the newest value in gait_selection."""
        self._use_position_queue = self.gait_selection.use_position_queue

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
                if self.subgait_id == "left_swing":
                    self._start_from_left_side = True
                else:
                    self._start_from_left_side = False
                self.logger.info(f"Starting subgait set to {self.subgait_id}")
            else:
                raise WrongStartPositionError(self.home_stand_position_all_joints, self.start_position_all_joints)
        except WrongStartPositionError as e:
            self.logger.warn(f"Can only change start side in home stand position. {e.msg}")

    def _predetermined_foot_location_callback(self, msg: String) -> None:
        self._use_predetermined_foot_location = True
        self._predetermined_foot_location = PREDETERMINED_FOOT_LOCATIONS[msg.data]
        self.logger.info(f"Stepping to stone {msg.data}")

    def _update_start_position_idle_state(self, joint_state: JointState) -> None:
        """Update the start position of the next subgait to be the last position of the previous gait."""
        for i, name in enumerate(self.all_joint_names):
            self.start_position_all_joints[name] = joint_state.position[i]
        self.start_position_actuating_joints = {
            name: self.start_position_all_joints[name] for name in self.actuating_joint_names
        }

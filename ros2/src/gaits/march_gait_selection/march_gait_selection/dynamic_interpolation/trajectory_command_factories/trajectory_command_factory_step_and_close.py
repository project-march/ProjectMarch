"""Author: Marten Haitjema, MVII."""

from typing import Dict, Optional

from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError, VelocitySoftLimitError
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)

from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import Point
from std_msgs.msg import Header


class TrajectoryCommandFactoryStepAndClose(TrajectoryCommandFactory):
    """Class that creates and validates a trajectory command for a step and close."""

    def __init__(self, gait, point_handler):
        super().__init__(gait, point_handler)
        self._gait = gait
        self._point_handler = point_handler
        self._logger = gait.node.get_logger().get_child(__class__.__name__)
        self._trajectory_failed = False

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
            self._gait._end = True
            self._logger.info("Stopping dynamic gait.")
        else:
            if self._use_position_queue:
                self.foot_location = self._get_foot_location_from_queue()
                if self.position_queue.empty():
                    self.fill_queue()
                    self._logger.warn(f"Queue is empty. Resetting queue to {list(self.position_queue.queue)}")
            else:
                try:
                    self.foot_location = self._point_handler.get_foot_location(self.subgait_id)
                    stop = self._point_handler.is_foot_location_too_old(self.foot_location)
                    self._point_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
                except AttributeError:
                    self._logger.warn("No FootLocation found. Connect the camera or use a gait with a fixed step size.")
                    self._gait._end = True
                    return None

        if not stop:
            self._point_handler.publish_chosen_foot_position(self.subgait_id, self.foot_location)
            self._logger.info(
                f"Stepping to location ({self.foot_location.processed_point.x}, "
                f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
            )

        return self._get_first_feasible_trajectory(start, stop)

    def _can_get_second_step(self, is_final_iteration: bool) -> bool:
        """Tries to create the subgait that is one step ahead, which is a stop gait for step and close.

        If this is not possible, the first subgait should not be executed.

        Args:
            is_final_iteration (bool): True if current iteration equals the maximum amount of iterations
        Returns:
            bool: true if second step close gait can be made.
        """
        start_position = self.dynamic_step.get_final_position()
        subgait_id = "right_swing" if self.subgait_id == "left_swing" else "left_swing"
        subgait = self._create_subgait_instance(
            start_position,
            subgait_id,
            start=False,
            stop=True,
        )
        try:
            subgait.get_joint_trajectory_msg(self._gait.add_push_off)
        except (PositionSoftLimitError, VelocitySoftLimitError) as e:
            if is_final_iteration:
                self._logger.warn(f"Close gait is not feasible. {e.msg}")
            return False
        return True

    def _get_foot_location_from_queue(self) -> FootPosition:
        """Get FootPosition message from the position queue.

        Returns:
            FootPosition: FootPosition msg with position from queue
        """
        header = Header(stamp=self._gait.node.get_clock().now().to_msg())
        point_from_queue = self.position_queue.get()
        point = Point(x=point_from_queue["x"], y=point_from_queue["y"], z=point_from_queue["z"])

        return FootPosition(header=header, processed_point=point, duration=self.duration_from_yaml)

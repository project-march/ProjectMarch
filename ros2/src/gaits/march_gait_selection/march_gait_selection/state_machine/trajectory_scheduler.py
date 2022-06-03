"""Author: ???."""

from __future__ import annotations

from queue import LifoQueue
from typing import List, Set

from attr import dataclass

from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_GetResult_Response
from rclpy import Future
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

from control_msgs.action import FollowJointTrajectory
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from march_shared_msgs.msg import (
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import JointTrajectory

TRAJECTORY_SCHEDULER_HISTORY_DEPTH = 5


@dataclass
class TrajectoryCommand:
    """A container for scheduling trajectories.

    It contains besides the trajectory to be scheduled some additional information
    about the scheduled trajectory, such as the subgait name, duration and start time
    of the trajectory.
    """

    trajectory: JointTrajectory
    duration: Duration
    name: str
    start_time: Time

    @staticmethod
    def from_subgait(subgait: Subgait, start_time: Time) -> TrajectoryCommand:
        """Create a TrajectoryCommand from a subgait.

        Args:
            subgait (Subgait): subgait to create a command from
            start_time (Time): time at which subgait should be scheduled
        Returns:
            TrajectoryCommand: command corresponding to the given subgait at the given start time
        """
        return TrajectoryCommand(
            subgait.to_joint_trajectory_msg(),
            subgait.duration,
            subgait.subgait_name,
            start_time,
        )

    def __str__(self) -> str:
        """Returns a string with the name, start time and duration."""
        return f"({self.name}, {self.start_time.nanoseconds}, {self.duration.nanoseconds})"


class TrajectoryScheduler:
    """Scheduler that sends the wanted trajectories to the topic listened to by the exoskeleton/simulation.

    Args:
        node (Node): node that is used to create subscribers/publishers
    Attributes:
        _logger (Logger): used to log to the terminal
        _failed (bool): ???
        _node (Node): node that is used to create subscribers/publishers
        _goals (List[TrajectoryCommand]): list containing trajectory commands
        _trajectory_goal_pub (Publisher): used to publish FollowJointTrajectoryActionGoal on
            /march/controller/trajectory/follow_joint_trajectory_goal
        _cancel_pub (Publisher): publishes a GoalID message on /march/controller/trajectory/follow_joint...
            ..._trajectory/goal
        _trajectory_goal_result_sub (Subscriber): Listens for FollowJointTrajectoryActionResult on
            "march/controller/trajectory/follow_joint_trajectory/result. Receives the errors
        _trajectory_command_pub (Publisher): publishes JointTrajectory messages on
            /march/controller/trajectory/command

        For more information on the Action client see these pages:
            * https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html?highlight=action
            * https://docs.ros.org/en/foxy/Tutorials/Actions/Creating-an-Action.html
            * https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html
    """

    def __init__(self, node: Node):
        self._failed = False
        self._node = node
        self._active_goals = LifoQueue()
        self._messages_in_transit: Set[Future] = set()
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._schedule_timeout = node.get_parameter("early_schedule_duration").get_parameter_value().double_value
        self._action_client = ActionClient(self._node, FollowJointTrajectory,
                                           "/joint_trajectory_controller/follow_joint_trajectory")

    def schedule(self, command: TrajectoryCommand) -> None:
        """Schedules a new trajectory.

        Args:
            command (TrajectoryCommand): The trajectory command to schedule
        """
        self._failed = False
        goal_msg = FollowJointTrajectory.Goal()
        command.trajectory.header.stamp = command.start_time.to_msg()  # To set early scheduling
        goal_msg.trajectory = command.trajectory
        if not self._action_client.wait_for_server(self._schedule_timeout):
            self._logger.warn(f"Failed to schedule trajectory {command} within {self._schedule_timeout} seconds")
            return

        goal_future: Future = self._action_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._gait_executor_start_request_cb)
        self._messages_in_transit.add(goal_future)

    def _gait_executor_start_request_cb(self, future: Future):
        self._messages_in_transit.remove(future)
        gait_executor_response_goal_handle: ClientGoalHandle = future.result()
        if not gait_executor_response_goal_handle.accepted:
            self._logger.warning("Goal message to execute gait was not accepted by the Action server (e.g. gazebo). "
                                 f"Goal: {gait_executor_response_goal_handle}")
            return

        self._active_goals.put(future)
        self._logger.info(f"Goal message to execute gait is accepted.")
        gait_execution_result_future: Future = gait_executor_response_goal_handle.get_result_async()
        gait_execution_result_future.add_done_callback(self._gait_executor_finished_cb)

    def _gait_executor_finished_cb(self, future: Future):
        """

        Args:
            future (Future[FollowJointTrajectory_GetResult_Response]):

        Returns:

        """
        result: FollowJointTrajectoryResult = future.result().result
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._logger.warning(f"Failed to execute trajectory: {result.error_string}")
            self._failed = True

    def cancel_active_goals(self) -> None:
        """Cancels the active goal."""
        for msg in self._messages_in_transit:
            msg.cancel()
        while not self._active_goals.empty():
            self._active_goals.get().result().cancel_goal_async()

    def failed(self) -> bool:
        """Returns true if the trajectory failed."""
        return self._failed

    def reset(self) -> None:
        """Reset attributes of class."""
        self._failed = False
        self._active_goals = LifoQueue()
        self._messages_in_transit.clear()

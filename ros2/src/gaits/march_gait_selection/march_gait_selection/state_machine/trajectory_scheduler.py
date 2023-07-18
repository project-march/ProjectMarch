"""Author: George Vegelien, MVII."""

from __future__ import annotations

from queue import LifoQueue
from typing import Set

from attr import dataclass

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

SCHEDULE_TIMEOUT = 0.1


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
    """Scheduler that handles the trajectory communication with the controller controlling the exoskeleton/simulation.

    Made and used only in the statemachine. Trajectories can be scheduled by calling the `schedule(...)` method.

    Args:
        node (Node): Node that is used to create subscribers/publishers.

    Attributes:
        _logger (Logger): Used to log to the terminal.
        _failed (bool): Checks if the last gait was successfully executed.
        _node (Node): Node that is used to create an action client.
        _active_goals (LifoQueue[Future[ClientGoalHandle]]): An Last in First out queue (aka 'stack') containing
            the already finished future objects containing as result an ClientGoalHandles. Which handles the
            active goals. These can be used to cancel the goal.
        _messages_in_transit (Set[Future]): An set of future object of messages containing a request to start
            a trajectory. These messages should be canceled if stopped. Otherwise, a message that is in transit can
            still arrive after all goals are canceled.
        _action_client_to_controller (ActionClient): The client that send the initial connection request with the
            controller server to send goals. Over the Action topic
            `/joint_trajectory_controller/follow_joint_trajectory`.

    More information about Actions:
        * https://docs.ros.org/en/foxy/Tutorials/Understanding-ROS2-Actions.html?highlight=action
        * https://docs.ros.org/en/foxy/Tutorials/Actions/Creating-an-Action.html
        * https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html
    """

    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._failed = False
        self._active_goals = LifoQueue()
        self._messages_in_transit: Set[Future] = set()
        self._action_client_to_controller = ActionClient(
            self._node, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )

    def schedule(self, command: TrajectoryCommand) -> None:
        """Sends a trajectory to the controller.

        If the `command` does not contain a `start_time` the gait will start as soon as the message arrives.

        This method will run `_gait_executor_start_request_cb(future)` if the requested trajectory is received at the
        controller.

        Args:
            command (TrajectoryCommand): The trajectory command to schedule.
        """
        self._failed = False
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = command.trajectory
        self._logger.info("command.trajectory:" + str(command.trajectory))

        if not self._action_client_to_controller.wait_for_server(SCHEDULE_TIMEOUT):
            self._logger.warn(f"Failed to schedule trajectory {command} within {SCHEDULE_TIMEOUT} seconds")
            return

        goal_future: Future = self._action_client_to_controller.send_goal_async(goal_msg)
        goal_future.add_done_callback(self._controller_starts_trajectory_cb)
        self._messages_in_transit.add(goal_future)

    def _controller_starts_trajectory_cb(self, future: Future) -> None:
        """Callback for if the trajectory start_request message has arrived at the controller.

        It will check if the trajectory is accepted:
            * If accepted it will store the goal_handle and start a call to `_controller_finished_trajectory_cb(future)`
                when the trajectory is done executing.
            * If not accepted, it will log a warning and do nothing. The trajectory will not be executed because
                it is canceled from the controllers side.

        Args:
            future (Future[ClientGoalHandle]): Finished future object containing as result an ClientGoalHandle.
        """
        self._messages_in_transit.remove(future)
        gait_executor_response_goal_handle: ClientGoalHandle = future.result()
        if not gait_executor_response_goal_handle.accepted:
            self._logger.warning(
                "Goal message to execute gait was not accepted by the Action server (e.g. gazebo). "
                f"Goal: {gait_executor_response_goal_handle}"
            )
            return

        self._active_goals.put(future)
        self._logger.info("Goal message to execute gait is accepted.")
        gait_execution_result_future: Future = gait_executor_response_goal_handle.get_result_async()
        gait_execution_result_future.add_done_callback(self._controller_finished_trajectory_cb)

    def _controller_finished_trajectory_cb(self, future: Future) -> None:
        """Callback for if the trajectory is finished.

        If the trajectory did not execute successfully it will set the `_failed` value to True, which can later be
        retrieved by the statemachine.

        Args:
            future (Future[FollowJointTrajectory_GetResult_Response]): A future object containing the result if
                the trajectory is successfully executed.
        """
        result: FollowJointTrajectoryResult = future.result().result
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._logger.error(f"Failed to execute trajectory: {result.error_string}")
            self._failed = True

    def cancel_active_goals(self) -> None:
        """Cancels the active goal.

        If active goal is canceled the controller will hold its position.
        """
        for msg in self._messages_in_transit:
            msg.cancel()
        while not self._active_goals.empty():
            self._active_goals.get().result().cancel_goal_async()

    def failed(self) -> bool:
        """Returns true if the trajectory failed."""
        return self._failed

    def reset(self) -> None:
        """Reset attributes of class.

        Called everytime after successfully executing a gait.
        """
        self._failed = False
        self._active_goals = LifoQueue()
        self._messages_in_transit.clear()

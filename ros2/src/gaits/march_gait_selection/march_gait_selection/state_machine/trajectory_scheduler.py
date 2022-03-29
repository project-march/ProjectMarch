from __future__ import annotations
from typing import List

from attr import dataclass
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from march_shared_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryResult,
)
from march_utility.utilities.logger import Logger
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
        return f"({self.name}, {self.start_time.nanoseconds}, {self.duration.nanoseconds})"


class TrajectoryScheduler:
    """Scheduler that sends the wanted trajectories to the topic listened
    to by the exoskeleton/simulation.

    Args:
        node (Node): node that is used to create subscribers/publishers
    Attributes:
        logger (Logger): used to log to the terminal
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
    """

    def __init__(self, node: Node):
        self._failed = False
        self._node = node
        self._goals: List[TrajectoryCommand] = []
        self.logger = Logger(self._node, __class__.__name__)

        # Temporary solution to communicate with ros1 action server, should
        # be updated to use ros2 action implementation when simulation is
        # migrated to ros2
        self._trajectory_goal_pub = self._node.create_publisher(
            msg_type=FollowJointTrajectoryActionGoal,
            topic="/march/controller/trajectory/follow_joint_trajectory/goal",
            qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        )

        self._cancel_pub = self._node.create_publisher(
            msg_type=GoalID,
            topic="/march/controller/trajectory/follow_joint_trajectory/cancel",
            qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        )

        self._trajectory_goal_result_sub = self._node.create_subscription(
            msg_type=FollowJointTrajectoryActionResult,
            topic="/march/controller/trajectory/follow_joint_trajectory/result",
            callback=self._done_cb,
            qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        )

        # Publisher for sending hold position mode
        self._trajectory_command_pub = self._node.create_publisher(
            msg_type=JointTrajectory,
            topic="/march/controller/trajectory/command",
            qos_profile=TRAJECTORY_SCHEDULER_HISTORY_DEPTH,
        )

    def schedule(self, command: TrajectoryCommand) -> None:
        """Schedules a new trajectory.

        Args:
            command (TrajectoryCommand): The trajectory command to schedule
        """
        self._failed = False
        stamp = command.start_time.to_msg()
        command.trajectory.header.stamp = stamp
        goal = FollowJointTrajectoryGoal(trajectory=command.trajectory)
        self._trajectory_goal_pub.publish(
            FollowJointTrajectoryActionGoal(
                header=Header(stamp=stamp),
                goal_id=GoalID(stamp=stamp, id=str(command)),
                goal=goal,
            )
        )
        info_log_message = f"Scheduling {command.name}"
        debug_log_message = f"Subgait {command.name} starts "
        if self._node.get_clock().now() < command.start_time:
            time_difference = Duration.from_ros_duration(command.start_time - self._node.get_clock().now())
            debug_log_message += f"in {round(time_difference.seconds, 3)}s"
        else:
            debug_log_message += "now"

        self._goals.append(command)
        self.logger.info(info_log_message)
        self.logger.debug(debug_log_message)

    def cancel_active_goals(self) -> None:
        """Cancels the active goal"""
        now = self._node.get_clock().now()
        for goal in self._goals:
            if goal.start_time + goal.duration > now:
                self._cancel_pub.publish(GoalID(stamp=goal.start_time.to_msg(), id=str(goal)))

    def send_position_hold(self) -> None:
        """Schedule empty JointTrajectory message to hold position. Used during force unknown"""
        self._trajectory_command_pub.publish(JointTrajectory())

    def failed(self) -> bool:
        """Returns true if the trajectory failed"""
        return self._failed

    def reset(self) -> None:
        """Reset attributes of class"""
        self._failed = False
        self._goals = []

    def _done_cb(self, result) -> None:
        """Callback for when a result is published."""
        if result.result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self.logger.error(
                f"Failed to execute trajectory. {result.result.error_string} ({result.result.error_code})"
            )
            self._failed = True

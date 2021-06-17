from typing import List

from attr import dataclass
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from march_shared_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryResult,
)
from trajectory_msgs.msg import JointTrajectory


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
    def from_subgait(subgait: Subgait, start_time: Time):
        """Create a TrajectoryCommand from a subgait."""
        return TrajectoryCommand(
            subgait.to_joint_trajectory_msg(),
            subgait.duration,
            subgait.subgait_name,
            start_time,
        )

    def __str__(self):
        return (
            f"({self.name}, {self.start_time.nanoseconds}, {self.duration.nanoseconds})"
        )


class TrajectoryScheduler:
    """Scheduler that sends sends the wanted trajectories to the topic listened
    to by the exoskeleton/simulation."""

    def __init__(self, node):
        self._failed = False
        self._node = node
        self._goals: List[TrajectoryCommand] = []

        # Temporary solution to communicate with ros1 action server, should
        # be updated to use ros2 action implementation when simulation is
        # migrated to ros2
        self._trajectory_goal_pub = self._node.create_publisher(
            msg_type=FollowJointTrajectoryActionGoal,
            topic="/march/controller/trajectory/follow_joint_trajectory/goal",
            qos_profile=5,
        )

        self._cancel_pub = self._node.create_publisher(
            msg_type=GoalID,
            topic="/march/controller/trajectory/follow_joint_trajectory/cancel",
            qos_profile=5,
        )

        self._trajectory_goal_result_sub = self._node.create_subscription(
            msg_type=FollowJointTrajectoryActionResult,
            topic="/march/controller/trajectory/follow_joint_trajectory/result",
            callback=self._done_cb,
            qos_profile=5,
        )

        # Publisher for sending hold position mode
        self._trajectory_command_pub = self._node.create_publisher(
            msg_type=JointTrajectory,
            topic="/march/controller/trajectory/command",
            qos_profile=5,
        )

    def schedule(self, command: TrajectoryCommand):
        """Schedules a new trajectory.
        :param TrajectoryCommand command: The trajectory command to schedule
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
            time_difference = Duration.from_ros_duration(
                command.start_time - self._node.get_clock().now()
            )
            debug_log_message += f"in {round(time_difference.seconds, 3)}s"
        else:
            debug_log_message += "now"

        self._goals.append(command)
        self._node.get_logger().info(info_log_message)
        self._node.get_logger().debug(debug_log_message)

    def cancel_active_goals(self):
        now = self._node.get_clock().now()
        for goal in self._goals:
            if goal.start_time + goal.duration > now:
                self._cancel_pub.publish(
                    GoalID(stamp=goal.start_time.to_msg(), id=str(goal))
                )

    def send_position_hold(self):
        self._trajectory_command_pub.publish(JointTrajectory())

    def failed(self) -> bool:
        return self._failed

    def reset(self):
        self._failed = False
        self._goals = []

    def _done_cb(self, result):
        if result.result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._node.get_logger().error(
                f"Failed to execute trajectory. {result.result.error_string} ({result.result.error_code})"
            )
            self._failed = True

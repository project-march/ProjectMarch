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
class ScheduleCommand:
    trajectory: JointTrajectory
    duration: Duration
    name: str
    start_time: Time

    @staticmethod
    def from_subgait(subgait: Subgait, start_time: Time):
        return ScheduleCommand(subgait.to_joint_trajectory_msg(), subgait.duration, subgait.subgait_name, start_time)


class TrajectoryScheduler(object):
    """Scheduler that sends sends the wanted trajectories to the topic listened
    to by the exoskeleton/simulation."""

    def __init__(self, node):
        self._failed = False
        self._node = node

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

    def schedule(self, command: ScheduleCommand):
        """Schedules a new trajectory.
        :param JointTrajectory trajectory: a trajectory for all joints to follow
        """
        self._failed = False
        stamp = command.start_time.to_msg()
        command.trajectory.header.stamp = stamp
        goal = FollowJointTrajectoryGoal(trajectory=command.trajectory)
        self._trajectory_goal_pub.publish(
            FollowJointTrajectoryActionGoal(
                header=Header(stamp=stamp), goal_id=GoalID(stamp=stamp), goal=goal
            )
        )
        self._node.get_logger().info(f"Scheduling {command.name}, "
                                     f"t_n={round(self._node.get_clock().now().nanoseconds / 1e9, 3)}, "
                                     f"t_s={round(command.start_time.nanoseconds / 1e9, 3)},"
                                     f"d={round(command.duration.nanoseconds / 1e9, 2)} ")

    def failed(self):
        return self._failed

    def reset(self):
        self._failed = False

    def _done_cb(self, result):
        if result.result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._node.get_logger().error(
                f"Failed to execute trajectory. {result.result.error_string} ({result.result.error_code})"
            )
            self._failed = True

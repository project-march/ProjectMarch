from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from march_shared_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryResult,
)


class TrajectoryScheduler(object):
    """Scheduler that sends sends the wanted trajectories to the topic listened
    to by the exoskeleton/simulation."""

    def __init__(self, node):
        self._failed = False
        self._node = node
        self._last_goal = None

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

    def schedule(self, trajectory):
        """Schedules a new trajectory.
        :param JointTrajectory trajectory: a trajectory for all joints to follow
        """
        self._failed = False
        stamp = self._node.get_clock().now().to_msg()
        goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        self._trajectory_goal_pub.publish(
            FollowJointTrajectoryActionGoal(
                header=Header(stamp=stamp), goal_id=GoalID(stamp=stamp), goal=goal
            )
        )

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

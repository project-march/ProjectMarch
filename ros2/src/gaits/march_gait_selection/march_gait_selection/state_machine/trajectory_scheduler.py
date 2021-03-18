from typing import Optional

from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy.time import Time
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from march_shared_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryActionResult,
    FollowJointTrajectoryResult
)
from trajectory_msgs.msg import JointTrajectory


class TrajectoryScheduler(object):
    """Scheduler that sends sends the wanted trajectories to the topic listened
    to by the exoskeleton/simulation."""

    def __init__(self, node):
        self._failed = False
        self._node = node
        self._last_goal = None

        # Values of previous scheduled trajectory
        self._previous_trajectory_end_time = None

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
        
        # self._trajectory_goal_result_sub = self._node.create_subscription(
        #     msg_type=FollowJointTrajectoryActionFeedback,
        #     topic="/march/controller/trajectory/follow_joint_trajectory/feedback",
        #     callback=self._feedback_cb,
        #     callback_group=ReentrantCallbackGroup,
        #     qos_profile=10,
        # )

        self._trajectory_goal_result_sub = self._node.create_subscription(
            msg_type=FollowJointTrajectoryActionResult,
            topic="/march/controller/trajectory/follow_joint_trajectory/result",
            callback=self._done_cb,
            qos_profile=5,
        )

    def schedule(self, trajectory: JointTrajectory, should_delay_start: Optional[bool] = False):
        """Schedules a new trajectory.
        :param JointTrajectory trajectory: a trajectory for all joints to follow
        :param should_delay_start: Flag indicating whether the trajectory should be scheduled
        with a delay of the previous subgait.
        """
        self._failed = False
        now = self._node.get_clock().now()
        if should_delay_start and self._can_delay_start():
            trajectory.header.stamp = self._previous_trajectory_end_time.to_msg()
            self._previous_trajectory_end_time += Duration.from_msg(trajectory.points[-1].time_from_start)
        else:
            start_time = now + Duration(seconds=0.3)
            self._previous_trajectory_end_time = start_time + Duration.from_msg(trajectory.points[-1].time_from_start)
            trajectory.header.stamp = start_time.to_msg()
            # self._previous_trajectory_end_time = now + Duration.from_msg(trajectory.points[-1].time_from_start)

        self._node.get_logger().info('Schedule time: ' +
                                     str(now.to_msg().sec + round(
                                         now.to_msg().nanosec / 1e9, 2)))
        self._node.get_logger().info('Trajectory start time: ' +
                                     str(trajectory.header.stamp.sec + round(
                                         trajectory.header.stamp.nanosec / 1e9, 2)))

        goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        self._trajectory_goal_pub.publish(
            FollowJointTrajectoryActionGoal(
                header=Header(stamp=now.to_msg()),
                goal_id=GoalID(stamp=now.to_msg()), goal=goal
            )
        )

    def reset_previous_trajectory_values(self):
        "Reset previous trajectory values"
        self._previous_trajectory_end_time = None

    def _can_delay_start(self):
        """Start can be delayed if both previous values are not None."""
        return self._previous_trajectory_end_time is not None

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

    # def _feedback_cb(self, feedback):
    #     pass

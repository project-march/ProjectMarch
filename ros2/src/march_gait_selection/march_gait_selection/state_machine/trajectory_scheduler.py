from rclpy.action import ActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult


class TrajectoryScheduler(object):
    def __init__(self, node):
        self._failed = False
        self._node = node
        self._trajectory_client = ActionClient(node, FollowJointTrajectoryAction, 'follow_joint_trajectory')

    def schedule(self, trajectory):
        """Schedules a new trajectory.

        :param JointTrajectory trajectory: a trajectory for all joints to follow
        """
        self._failed = False
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._trajectory_client.send_goal(goal, done_cb=self._done_cb)

    def failed(self):
        return self._failed

    def reset(self):
        self._failed = False

    def _done_cb(self, _state, result):
        if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            self._node.get_logger().err('Failed to execute trajectory. {0} ({1})'.format(result.error_string,
                                                                                         result.error_code))
            self._failed = True

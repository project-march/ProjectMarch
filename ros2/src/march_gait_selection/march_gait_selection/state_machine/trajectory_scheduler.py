from std_msgs.msg import Header
from actionlib_msgs.msg import GoalID
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from march_shared_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryActionGoal


class TrajectoryScheduler(object):
    def __init__(self, node):
        self._failed = False
        self._node = node
        self._trajectory_client = ActionClient(node, FollowJointTrajectory, 'follow_joint_trajectory')
        self._trajectory_goal_pub = self._node.create_publisher(
            msg_type=FollowJointTrajectoryActionGoal,
            topic='/march/controller/trajectory/follow_joint_trajectory/goal',
            qos_profile=5)

    def schedule(self, trajectory):
        """Schedules a new trajectory.

        :param JointTrajectory trajectory: a trajectory for all joints to follow
        """
        self._failed = False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        result = self._trajectory_client.send_goal_async(goal)
        self._node.get_logger().info('hey, i send a goal!')
        ros1_goal = FollowJointTrajectoryGoal(trajectory=trajectory)
        self._trajectory_goal_pub.publish(FollowJointTrajectoryActionGoal(
            header=Header(stamp=self._node.get_clock().now().to_msg()), goal_id=GoalID(), goal=ros1_goal))
        result.add_done_callback(self._done_cb)

    def failed(self):
        return self._failed

    def reset(self):
        self._failed = False

    def _done_cb(self, _state, result):
        if result.error_code != FollowJointTrajectory.Result().SUCCESSFUL:
            self._node.get_logger().err('Failed to execute trajectory. {0} ({1})'.format(result.error_string,
                                                                                         result.error_code))
            self._failed = True

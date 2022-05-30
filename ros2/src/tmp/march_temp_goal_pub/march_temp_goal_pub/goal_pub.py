import rclpy
import std_msgs.msg
import trajectory_msgs.msg
from builtin_interfaces.msg import Duration
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from std_msgs.msg import String, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import control_msgs.action
import march_utility.utilities.utility_functions
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('temp_goal_publisher')
        self.get_logger().info("Started the node.")
        self.create_subscription(Float64, "/walk_to", self.send_goal_callback, 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory")

    def send_goal_callback(self, msg: Float64):
        position = msg.data
        self.get_logger().info(f"Waling to: {position}")
        goal_msg = FollowJointTrajectory.Goal()

        joints = march_utility.utilities.utility_functions.get_joint_names_from_urdf()

        point1 = JointTrajectoryPoint()
        point1.time_from_start = Duration(sec=0)
        point1.positions = [0.0] * len(joints)
        # point1.velocities = [0.0] * len(joints)
        # point1.accelerations = [0.0] * len(joints)
        # point1.effort = [0.0] * len(joints)

        point2 = JointTrajectoryPoint()
        point2.time_from_start = Duration(sec=2)
        point2.positions = [position] * len(joints)
        # point2.velocities = [0.0] * len(joints)
        # point2.accelerations = [0.0] * len(joints)
        # point2.effort = [0.0] * len(joints)

        trajectory_msg = JointTrajectory(
            joint_names=joints,
            points=[point1, point2]
        )
        self.action_client.wait_for_server()

        goal_msg.trajectory = trajectory_msg
        goal_msg.goal_tolerance = [JointTolerance(position=1.0)] * len(joints)

        self.get_logger().info(f"Starting the goal, for joint names: {joints}, to positions: {point2.positions}")
        res = self.action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Moved to: {res}")



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

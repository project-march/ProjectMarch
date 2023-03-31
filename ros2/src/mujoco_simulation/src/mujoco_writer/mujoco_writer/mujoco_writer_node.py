"""Author: MVIII."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool

from mujoco_interfaces.msg import MujocoInput


class MujocoWriterNode(Node):
    """Writer node that writes the data from the MARCH code to the Mujoco simulation.

    The node gets data from the correct topic, and passes that data to the correct topic for mujoco_sim to use.
    """

    def __init__(self):
        """This node is responsible for sending any commands from our ROS systems to the Mujoco Sim node.

        This node subscribes to the joint_trajectory_state/state topic.
        On this topic, the MARCH Ros system publishes the actual and desired states,
        that the low level controllers should work with.
        This node is a passthrough from the MARCH state messages to the Mujoco sim node.
        """
        super().__init__("mujoco_writer")
        self.publisher = self.create_publisher(MujocoInput, "mujoco_input", 10)
        # self.subscription = self.create_subscription(
        #     JointTrajectoryControllerState, "joint_trajectory_controller/state", self.listener_callback, 10
        # )

        # A subscriber that notifies if the queue with trajectory points has to  be reset.
        self.reset_subscription = self.create_subscription(
            Bool, "/march/mujoco_reset_trajectory", self.reset_callback, 10
        )

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback)
        # self._action_server.register_goal_callback(self.goal_callback)
        self.reset = False
        # self.subscription  # prevent unused variable warning

    def goal_callback(self, goal):
        if len(goal.trajectory.joint_names) == 0:
            return GoalResponse.REJECT
        self.get_logger().info("Accepted new action goal")
        return GoalResponse.ACCEPT_AND_EXECUTE


    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        trajectory = goal_handle.request.trajectory.points
        self.get_logger().info('amount of points: ' + str(len(trajectory)))
        # for msg in trajectory:
        #     msg_tosend = MujocoInput()
        #     skip = False
        #     for i, x in enumerate(msg.positions):
        #         if x != x:
        #             skip = False
        #             break
        #         else:
        #             msg.positions[i] *= 1
        #     if not skip:
        #         msg_tosend.trajectory = JointTrajectoryControllerState()
        #         msg_tosend.trajectory.desired = msg
        #         if self.reset:
        #             msg_tosend.reset = 1
        #             self.reset = False
        #         self.publisher.publish(msg_tosend)
        msg_to_send = MujocoInput()
        msg_to_send.points = trajectory
        self.publisher.publish(msg_to_send)
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

    def listener_callback(self, msg):
        """This listener callback publishes all the messages from the MARCH code to the topic Mujoco sim subscribes to.

        This callback is just a simple passthrough to keep the flow clear.
        """
        msg_tosend = MujocoInput()
        skip = False
        for i, x in enumerate(msg.desired.positions):
            if x != x:
                skip = True
                break
            else:
                msg.desired.positions[i] *= 1
        if not skip:
            msg_tosend.trajectory = msg
            if self.reset:
                msg_tosend.reset = 1
                self.reset = False
            self.publisher.publish(msg_tosend)

    def reset_callback(self, msg):
        """Set the reset flag when a message is received with data True."""
        self.reset = msg.data


def main(args=None):
    """Main function for life cycle of the node.

    :param args:
    :return:
    """
    rclpy.init(args=args)
    node = MujocoWriterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

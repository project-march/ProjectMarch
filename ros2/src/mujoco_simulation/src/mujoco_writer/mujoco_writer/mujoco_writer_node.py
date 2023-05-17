"""Author: MVIII."""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from rclpy.action import ActionServer
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
        self.subscription = self.create_subscription(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", self.listener_callback, 10
        )

        # A subscriber that notifies if the queue with trajectory points has to  be reset.
        self.reset_subscription = self.create_subscription(
            Bool, "/march/mujoco_reset_trajectory", self.reset_callback, 10
        )

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            self.execute_callback)
        self.reset = False

    def execute_callback(self, goal_handle):
        """Callback for the joint_trajectory action server."""
        trajectory = goal_handle.request.trajectory.points
        msg_to_send = MujocoInput()
        msg_to_send.points = trajectory
        self.publisher.publish(msg_to_send)
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

    def listener_callback(self, msg):
        """This listener callback publishes all the messages from the MARCH code to the topic Mujoco sim subscribes to.

        This callback is just a simple passthrough to keep the flow clear.
        """
        trajectory = msg.points
        msg_to_send = MujocoInput()
        msg_to_send.points = trajectory
        self.publisher.publish(msg_to_send)

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

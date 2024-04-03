"""Author: MVIII."""

import math
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory

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
        #     JointTrajectoryControllerState, "joint_trajectory_controller/state", self.listener_callback, 100
        # )

        # A subscriber that notifies if the queue with trajectory points has to  be reset.
        self.reset_subscription = self.create_subscription(Bool, "/mujoco_reset_trajectory", self.reset_callback, 10)
        self.reset = False

        self.subscription = self.create_subscription(
            JointTrajectory, "joint_trajectory_controller/joint_trajectory", self.listener_callback_point, 100
        )

    def listener_callback_point(self, msg):
        """This listener callback publishes all the messages from the MARCH code to the topic Mujoco sim subscribes to.

        This callback is just a simple passthrough to keep the flow clear.
        """
        msg_to_send = MujocoInput()
        msg_to_send.points = msg.points
        msg_to_send.joint_names = msg.joint_names
        
        if self.reset:
            msg_to_send.reset = 1
            self.reset = False
        self.publisher.publish(msg_to_send)

        # if msg.points:
        #     if not any(math.isnan(x) for x in msg.points[0]):
        #         msg_to_send.points = msg.points

        #         if self.reset:
        #             msg_to_send.reset = 1
        #             self.reset = False

        #         self.publisher.publish(msg_to_send)

    def listener_callback(self, msg):
        """This listener callback publishes all the messages from the MARCH code to the topic Mujoco sim subscribes to.

        This callback is just a simple passthrough to keep the flow clear.
        """
        msg_to_send = MujocoInput()
        skip = False
        if len(msg.desired.positions) == 0:
            skip = True
        for i, x in enumerate(msg.desired.positions):
            if x != x:
                skip = True
                break
            else:
                msg.desired.positions[i] *= 1
        if not skip:
            msg_to_send.trajectory = msg
            if self.reset:
                msg_to_send.reset = 1
                self.reset = False
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

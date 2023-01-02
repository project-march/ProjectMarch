"""Author: MVIII."""

import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
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
        self.publisher = self.create_publisher(MujocoInput, 'mujoco_input', 10)
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            'joint_trajectory_controller/state',
            self.listener_callback,
            10)
        # self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """This listener callback publishes all the messages from the MARCH code to the topic Mujoco sim subscribes to.

        This callback is just a simple passthrough to keep the flow clear.
        """
        msg_tosend = MujocoInput()
        msg_tosend.trajectory = msg
        msg_tosend.reset = 0
        self.publisher.publish(msg_tosend)


def main(args=None):
    """Main function for life cycle of the node.

    :param args:
    :return:
    """
    rclpy.init(args=args)
    node = MujocoWriterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

"""Author: MVIII."""

from mujoco_interfaces.msg import MujocoDataState
from mujoco_interfaces.msg import MujocoDataSensing
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node


def convert_mujoco_data_state_to_joint_state(msg):
    """Converts the MojocoDataState to a JointState message."""
    joint_state = JointState()
    joint_state.name = msg.names
    joint_state.position = msg.qpos
    joint_state.velocity = msg.qvel
    joint_state.effort = msg.qacc
    return joint_state


class MujocoReaderNode(Node):
    """This node read all the state and sensor data from the Mujoco sim.

    That data is published on a topic that the March HWI can access.
    """

    def __init__(self):
        """This node is responsible for obtaining data from Mujoco.

        Creates all data needed in the node object.
        """
        super().__init__("mujoco_reader")
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.sensor_publisher = self.create_publisher(MujocoDataSensing, 'mjc_exo_sensing', 10)
        self.state_subscription = self.create_subscription(
            MujocoDataState,
            'mujoco_state_output',
            self.state_listener_callback,
            10)
        self.state_subscription  # prevent unused variable warning
        self.sensor_subscription = self.create_subscription(
            MujocoDataState,
            'mujoco_sensor_output',
            self.sensor_listener_callback,
            10)
        self.sensor_subscription  # prevent unused variable warning

    # Now the callback just plainly passes through the message, later on it might be the case that it has to be
    # converted to another message type
    def state_listener_callback(self, msg):
        """Listens to mujoco_state_output topic, and retrieves all newly published messages.

        These messages are converted to a joint_state msg, an published on hte joint_state topic.
        :param msg: a msg of mujoco_state_output type
        :return: None
        """
        self.state_publisher.publish(convert_mujoco_data_state_to_joint_state(msg))

    # Now the callback just plainly passes through the message, later on it might be the case that it has to be
    # converted to another message type
    def sensor_listener_callback(self, msg):
        """Listens to mujoco_sensor_output topic, and retrieves all newly published messages.

        These messages are converted to a MujocoDataSensing msg, an published on hte mjc_exo_sensing topic.
        :param msg: a msg of MujocoDataSensing type
        :return: None
        """
        self.sensor_publisher.publish(msg)


def main(args=None):
    """Main function of the node.

    :param args:
    :return:
    """
    rclpy.init(args=args)
    node = MujocoReaderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

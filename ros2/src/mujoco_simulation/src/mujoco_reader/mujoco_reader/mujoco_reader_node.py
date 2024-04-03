"""Author: MVIII."""

from mujoco_interfaces.msg import MujocoDataSensing
from sensor_msgs.msg import JointState, Imu
import rclpy
from rclpy.node import Node


def convert_mujoco_data_state_to_joint_state(msg):
    """Converts the JointState to a JointState message."""
    joint_state = JointState()
    joint_state.header.stamp = rclpy.get_clock().now().to_msg()
    joint_state.header.frame_id = "joint_link"
    joint_state.name = msg.name
    joint_state.position = msg.position
    joint_state.velocity = msg.velocity
    joint_state.effort = msg.effort
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
        self.state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.torso_imu_publisher = self.create_publisher(Imu, "upper_imu", 10)
        self.backpack_imu_publisher = self.create_publisher(Imu, "lower_imu", 10)

        self.sensor_subscription = self.create_subscription(
            MujocoDataSensing, "mujoco_sensor_output", self.sensor_listener_callback, 100
        )

    def sensor_listener_callback(self, msg):
        """Listens to mujoco_sensor_output topic, and retrieves all newly published messages.

        These messages are converted to a MujocoDataSensing msg, an published on hte mjc_exo_sensing topic.
        :param msg: a msg of MujocoDataSensing type
        :return: None
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = "joint_link"
        # joint_state.name = msg.joint_state.name 
        joint_state.name = msg.joint_state.name + ["left_ankle_ie", "right_ankle_ie"]
        joint_state.position = msg.joint_state.position
        joint_state.velocity = msg.joint_state.velocity
        joint_state.effort = msg.joint_state.effort.tolist() + [0.0, 0.0]
        self.state_publisher.publish(joint_state)
        backpack_imu = msg.backpack_imu
        backpack_imu.header.stamp = self.get_clock().now().to_msg()
        backpack_imu.header.frame_id = "imu_link"
        self.backpack_imu_publisher.publish(backpack_imu)
        torso_imu = msg.torso_imu
        torso_imu.header.stamp = self.get_clock().now().to_msg()
        torso_imu.header.frame_id = "imu_link"
        self.torso_imu_publisher.publish(torso_imu)
    


def main(args=None):
    """Main function of the node.

    :param args:
    :return:
    """
    rclpy.init(args=args)
    node = MujocoReaderNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

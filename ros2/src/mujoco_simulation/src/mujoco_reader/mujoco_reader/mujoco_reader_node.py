"""Author: MVIII."""

from mujoco_interfaces.msg import MujocoDataSensing
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PointStamped
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
        self.backpack_position_publisher = self.create_publisher(PointStamped, "lower_imu/position", 10)

        self.sensor_subscription = self.create_subscription(
            MujocoDataSensing, "mujoco_sensor_output", self.sensor_listener_callback, 100
        )

    def sensor_listener_callback(self, msg):
        """Listens to mujoco_sensor_output topic, and retrieves all newly published messages.

        These messages are converted to a MujocoDataSensing msg, an published on hte mjc_exo_sensing topic.
        :param msg: a msg of MujocoDataSensing type
        :return: None
        """
        self.state_publisher.publish(msg.joint_state)
        backpack_imu = msg.backpack_imu
        backpack_imu.header.stamp = self.get_clock().now().to_msg()
        backpack_imu.header.frame_id = "imu_link"
        self.backpack_imu_publisher.publish(backpack_imu)
        torso_imu = msg.torso_imu
        torso_imu.header.stamp = self.get_clock().now().to_msg()
        torso_imu.header.frame_id = "imu_link"
        self.torso_imu_publisher.publish(torso_imu)

        backpack_position = PointStamped()
        backpack_position.header.stamp = self.get_clock().now().to_msg()
        backpack_position.header.frame_id = "world"
        backpack_position.point = msg.backpack_pos
        self.backpack_position_publisher.publish(backpack_position)

        names = ["l_heel_right", "l_heel_left", "l_met1", "l_hallux", "l_met3", "l_toes", "l_met5", "l_arch",
                 "r_heel_right", "r_heel_left", "r_met1", "r_hallux", "r_met3", "r_toes", "r_met5", "r_arch"]
    


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

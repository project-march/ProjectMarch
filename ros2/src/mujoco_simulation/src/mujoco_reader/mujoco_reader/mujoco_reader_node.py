import numpy as np
from mujoco_interfaces.srv import ReadMujoco
from mujoco_interfaces.msg import MujocoDataRequest

from mujoco_interfaces.msg import MujocoDataState
from mujoco_interfaces.msg import MujocoDataSensing
import rclpy
from rclpy.node import Node

class Mujoco_readerNode(Node):

    def __init__(self):
        """This node is responsible for obtaining data from Mujoco.
        NOTE: Right now, it only obtains Pose() type messages, but we
        can extend this to be more universal/modular in what data we want
        to obtain.
        """
        super().__init__("mujoco_reader")
        self.state_publisher = self.create_publisher(MujocoDataState, 'mjc_exo_state', 10)
        self.sensor_publisher = self.create_publisher(MujocoDataSensing, 'mjc_exo_sensing', 10)
        CONTROL_PUBLISH_RATE = 0.5
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
        self.state_publisher.publish(msg)

    # Now the callback just plainly passes through the message, later on it might be the case that it has to be
    # converted to another message type
    def sensor_listener_callback(self, msg):
        self.sensor_publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = Mujoco_readerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
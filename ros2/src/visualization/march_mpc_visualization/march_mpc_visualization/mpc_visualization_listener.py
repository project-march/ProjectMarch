import rclpy
from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg


class MpcListener(Node):
    def __init__(self):
        super().__init__('mpc_listener')
        self.subscription = self.create_subscription(MpcMsg, '/march/mpc', self.listener_callback, 1)
        self.number_of_joints = 8
        self.new_data_position = 0.0

    # Set all data
    def listener_callback(self, msg):
        self.new_data_position = msg.joint[0].estimation.states[0].array[0]
        self.get_logger().info('I heard: "%f"' % self.new_data_position)

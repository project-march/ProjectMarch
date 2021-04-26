import rclpy
from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg


class MpcListener(Node):
    def __init__(self):
        super().__init__('mpc_listener')
        #self.subscription = self.create_subscription(MpcMsg, '/march/mpc', self.listener_callback, 1)
        #self.subscription.
        self.publisher = self.create_publisher(MpcMsg, '/march/mpc', 10)
        msg = MpcMsg()
        msg.header.frame_id = 'base_link'
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('I have received data!')


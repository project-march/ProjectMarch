"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksFootPositions
from march_shared_msgs.msg import StateEstimation
from geometry_msgs.msg import Point

class IkTestNode(Node):

    def __init__(self) -> None:
        super().__init__('ik_test_node')

        self.state_estimation_subscriber = self.create_subscription(
            StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)
        self.iks_foot_positions_publisher = self.create_publisher(
            IksFootPositions, 'ik_solver/buffer/input', 10)
        
        self.left_foot = Point()
        self.left_foot.x = 0.175
        self.left_foot.y = 0.14
        self.left_foot.z = -0.9

        self.right_foot = Point()
        self.right_foot.x = 0.175
        self.right_foot.y = -0.14
        self.right_foot.z = -0.9
        
        self.get_logger().info('ik_test_node started, sending following information: ')
        for name, pos in [("Left ankle", self.left_foot), ("Right ankle", self.right_foot)]:
            self.get_logger().info(f"{name} foot at x={pos.x}, y={pos.y}, z={pos.z}")

    def state_estimation_callback(self, msg):
        iks_foot_positions_msg = IksFootPositions()
        iks_foot_positions_msg.header.stamp = self.get_clock().now().to_msg()
        iks_foot_positions_msg.left_foot_position = self.left_foot
        iks_foot_positions_msg.right_foot_position = self.right_foot
        self.iks_foot_positions_publisher.publish(iks_foot_positions_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IkTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

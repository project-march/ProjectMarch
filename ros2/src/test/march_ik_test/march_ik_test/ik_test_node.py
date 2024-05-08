"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.node import Node

from march_shared_msgs.msg import IksCommand
from march_shared_msgs.msg import IksFootPositions
from march_shared_msgs.msg import StateEstimation
from geometry_msgs.msg import Point

class IkTestNode(Node):

    def __init__(self) -> None:
        super().__init__('ik_test_node')

        self.state_estimation_subscriber = self.create_subscription(
            StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)
        self.iks_command_publisher = self.create_publisher(
            IksCommand, 'ik_solver/command', 10)
        self.iks_foot_positions_publisher = self.create_publisher(
            IksFootPositions, 'ik_solver/buffer/input', 10)
        
        self.left_foot = Point()
        self.left_foot.x = 0.19
        self.left_foot.y = 0.5
        self.left_foot.z = -0.7

        self.right_foot = Point()
        self.right_foot.x = 0.19
        self.right_foot.y = -0.0
        self.right_foot.z = -0.9

        self.current_feet_positions = None
        self.interpolation_duration = 5.0
        self.counter = 0.0

        iks_command_msg = IksCommand()
        iks_command_msg.header.stamp = self.get_clock().now().to_msg()
        iks_command_msg.exo_mode = "Stand"
        iks_command_msg.task_names = ["posture", "motion"]

        self.get_logger().info('ik_test_node started, sending following information: ')
        self.get_logger().info(f"Exo mode: {iks_command_msg.exo_mode} with task names: {iks_command_msg.task_names}")
        for name, pos in [("Left ankle", self.left_foot), ("Right ankle", self.right_foot)]:
            self.get_logger().info(f"{name} foot at x={pos.x}, y={pos.y}, z={pos.z}")
        self.iks_command_publisher.publish(iks_command_msg)

    def state_estimation_callback(self, msg):
        if self.current_feet_positions is None:
            self.current_feet_positions = IksFootPositions()
            self.current_feet_positions.left_foot_position.x = msg.body_ankle_pose[0].position.x
            self.current_feet_positions.left_foot_position.y = msg.body_ankle_pose[0].position.y
            self.current_feet_positions.left_foot_position.z = msg.body_ankle_pose[0].position.z
            self.current_feet_positions.right_foot_position.x = msg.body_ankle_pose[1].position.x
            self.current_feet_positions.right_foot_position.y = msg.body_ankle_pose[1].position.y
            self.current_feet_positions.right_foot_position.z = msg.body_ankle_pose[1].position.z

        interpolation_ratio = self.calculate_interpolate_ratio(msg.step_time)
        iks_foot_positions_msg = IksFootPositions()
        iks_foot_positions_msg.header.stamp = self.get_clock().now().to_msg()
        iks_foot_positions_msg.left_foot_position = self.interpolate_foot_position(
            self.current_feet_positions.left_foot_position, self.left_foot, interpolation_ratio)
        iks_foot_positions_msg.right_foot_position = self.interpolate_foot_position(
            self.current_feet_positions.right_foot_position, self.right_foot, interpolation_ratio)
        self.iks_foot_positions_publisher.publish(iks_foot_positions_msg)

    def calculate_interpolate_ratio(self, step_time):
        interpolation_ratio = min(self.counter / self.interpolation_duration, 1.0)
        self.counter += step_time
        return interpolation_ratio**3 / (interpolation_ratio**3 + (1 - interpolation_ratio)**3)
    
    def interpolate(self, start, end, ratio):
        return ratio * end + (1 - ratio) * start
    
    def interpolate_foot_position(self, start, end, ratio):
        interpolated_foot = Point()
        interpolated_foot.x = self.interpolate(start.x, end.x, ratio)
        interpolated_foot.y = self.interpolate(start.y, end.y, ratio)
        interpolated_foot.z = self.interpolate(start.z, end.z, ratio)
        return interpolated_foot

def main(args=None):
    rclpy.init(args=args)
    node = IkTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

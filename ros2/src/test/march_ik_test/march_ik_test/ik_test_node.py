"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.node import Node

import threading
import sys
from PyQt5.QtWidgets import QApplication
from .window import Window

from march_shared_msgs.msg import IksCommand
from march_shared_msgs.msg import IksFootPositions
from march_shared_msgs.msg import StateEstimation
from geometry_msgs.msg import Point

class IkTestNode(Node):

    def __init__(self, window) -> None:
        super().__init__('ik_test_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('interval', 5.0),
                ('left_foot.x', 0.19),
                ('left_foot.y', 0.16),
                ('left_foot.z', -0.87),
                ('right_foot.x', 0.19),
                ('right_foot.y', -0.16),
                ('right_foot.z', -0.87),
        ])

        self.state_estimation_subscriber = self.create_subscription(
            StateEstimation, 'state_estimation/state', self.state_estimation_callback, 10)
        self.iks_command_publisher = self.create_publisher(
            IksCommand, 'ik_solver/command', 10)
        self.iks_foot_positions_publisher = self.create_publisher(
            IksFootPositions, 'ik_solver/buffer/input', 10)
        
        self.left_foot = Point()
        self.left_foot.x = self.get_parameter('left_foot.x').value
        self.left_foot.y = self.get_parameter('left_foot.y').value
        self.left_foot.z = self.get_parameter('left_foot.z').value

        self.right_foot = Point()
        self.right_foot.x = self.get_parameter('right_foot.x').value
        self.right_foot.y = self.get_parameter('right_foot.y').value
        self.right_foot.z = self.get_parameter('right_foot.z').value

        self.current_feet_positions = None
        self.interpolation_duration = self.get_parameter('interval').value
        self.counter = 0.0

        iks_command_msg = IksCommand()
        iks_command_msg.header.stamp = self.get_clock().now().to_msg()
        iks_command_msg.exo_mode = "Stand"
        iks_command_msg.task_names = ["posture", "motion"]
        self.iks_command_publisher.publish(iks_command_msg)

        self.window = window

        self.get_logger().info('ik_test_node started, sending following information: ')
        self.get_logger().info(f"Exo mode: {iks_command_msg.exo_mode} with task names: {iks_command_msg.task_names}")
        for name, pos in [("Left ankle", self.left_foot), ("Right ankle", self.right_foot)]:
            self.get_logger().info(f"{name} foot at x={pos.x}, y={pos.y}, z={pos.z}")

    def state_estimation_callback(self, msg):
        if not self.window.is_publishing_:
            return

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
    # Create the application
    app = QApplication(sys.argv)

    # Create the window and display
    window = Window()
    window.show()

    # Initialize ROS2 and the node
    rclpy.init(args=args)
    node = IkTestNode(window=window)

    # Create the ROS2 spin thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

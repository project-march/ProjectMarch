"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
import threading
import sys
from PyQt5.QtWidgets import QApplication
from rclpy.node import Node
from mujoco_interfaces.msg import MujocoGains

from .window import Window

class MujocoPidTunerNode(Node):

    def __init__(self) -> None:
        super().__init__('mujoco_pid_tuner_node')

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.pid_gains_publisher = self.create_publisher(MujocoGains, 'mujoco_gains', 10)

        self.get_logger().info('Mujoco PID Tuner Node has been initialized.')

        self.get_gains = None

    def timer_callback(self) -> None:
        if self.get_gains is not None:
            kp, kd, ki = self.get_gains()
            self.publish_gains(kp, kd, ki)

    def publish_gains(self, kp, kd, ki) -> None:
        msg = MujocoGains()
        msg.controller_mode = 0 # Fixed at 0 for now
        msg.proportional_gains = kp
        msg.derivative_gains = kd
        msg.integral_gains = ki
        self.pid_gains_publisher.publish(msg)

    def configure_callback(self, get_gains) -> None:
        self.get_gains = get_gains

def ros_spin(node):
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    # Initialize ROS2 and the node
    rclpy.init()
    mujoco_pid_tuner_node = MujocoPidTunerNode()

    # Create the application
    app = QApplication(sys.argv)

    # Create the ROS2 spin thread
    ros_thread = threading.Thread(target=ros_spin, args=(mujoco_pid_tuner_node,), daemon=True)
    ros_thread.start()

    # Create the window and show it
    window = Window()
    window.configure_ui()
    window.show()

    # Set the callback for the node
    mujoco_pid_tuner_node.configure_callback(window.get_gains)

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
"""
Project MARCH IX, 2023-2024
Author: Alexander James Becoy @alexanderjamesbecoy
"""

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import signal
import threading
import sys
from PyQt5.QtWidgets import QApplication, QMessageBox
from PyQt5.QtGui import QIcon
from .window import Window, WindowState

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class TestJointsGuiNode(Node):

    def __init__(self, window: Window) -> None:
        super().__init__('test_joints_gui')
        self.window = window
        self.window.set_publish_callback(self.publish_joint_positions)
        self.timer_period = 0.05 # seconds

        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.desired_joint_positions_pub = self.create_publisher(Float64MultiArray, 'march_joint_position_controller/commands', 10)
        self.wall_timer = self.create_timer(self.timer_period, self.timer_callback)

        self.joint_state_msg = None
        self.get_logger().info("Test Joints GUI Node has been initialized.")
        self.get_logger().info("Waiting for joint state message to initialize GUI...")

    def joint_state_callback(self, msg: JointState) -> None:
        # Update with the new joint state message
        self.joint_state_msg = msg

    def timer_callback(self) -> None:
        if self.joint_state_msg is None:
            self.get_logger().warn("No joint state message received yet", throttle_duration_sec=5.0)
            return

        if self.window.joints is not None:
            self.window.update_actual_positions(self.joint_state_msg.name, self.joint_state_msg.position)
        
        # # Publish the desired joint positions
        # if self.window.is_running:
        #     if len(self.window.linear_desired_positions) > 0:
        #         joint_positions = self.window.linear_desired_positions.pop(0)
        #         self.publish_joint_positions(joint_positions)
        #     else:
        #         joint_positions = self.window.get_desired_positions()
        #         self.publish_joint_positions(joint_positions)

        if self.window.state == WindowState.RUNNING:
            joint_positions = self.window.get_desired_positions()
            self.publish_joint_positions(joint_positions)
        elif self.window.state == WindowState.TARGETS_SET:
            if len(self.window.linear_desired_positions) > 0:
                joint_positions = self.window.linear_desired_positions.pop(0)
                self.publish_joint_positions(joint_positions)
            else:
                self.window.is_running = False

    def publish_joint_positions(self, joint_positions: list) -> None:
        desired_joint_positions = Float64MultiArray()
        desired_joint_positions.data = joint_positions
        self.desired_joint_positions_pub.publish(desired_joint_positions)


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    if QMessageBox.question(None, '', "Are you sure you want to quit?",
                            QMessageBox.Yes | QMessageBox.No,
                            QMessageBox.No) == QMessageBox.Yes:
        rclpy.shutdown()
        QApplication.quit()
        sys.exit(0)


def main(args=None):    
    # # Register the SIGINT handler
    # signal.signal(signal.SIGINT, sigint_handler)

    # Create the application
    app = QApplication(sys.argv)
    icon_path = get_package_share_directory('march_test_joints_gui') + '/media/icon.png'
    app.setWindowIcon(QIcon(icon_path))

    # Create the window and display
    window = Window()

    # Initialize ROS2 and the node
    rclpy.init(args=args)
    node = TestJointsGuiNode(window=window)

    # Collect the joint names by reading the first joint state message
    while rclpy.ok() and node.joint_state_msg is None:
        rclpy.spin_once(node)

    # Create layout for each joint
    filtered_joint_names = [name for name in node.joint_state_msg.name if 'ie' not in name]
    window.create_layout(filtered_joint_names, 2.0 / node.timer_period) # Avoid signal aliasing according to Nyquist theorem
    window.show()

    # Create the ROS2 spin thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node))
    ros_thread.start()

    # try:
    #     while rclpy.ok():
    #         app.processEvents()
    # except KeyboardInterrupt:
    #     rclpy.shutdown()

    sys.exit(app.exec_())
    

if __name__ == '__main__':
    main()

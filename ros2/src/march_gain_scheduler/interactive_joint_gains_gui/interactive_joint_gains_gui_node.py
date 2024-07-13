import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QSizePolicy
from PyQt5.QtCore import QTimer  

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import JointState
from march_shared_msgs.msg import JointGains, JointMotorControllerState

CHECK_MESSAGE_STATUS_INTERVAL = 5000  # milliseconds

class InteractiveJointGainsGuiNode(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'interactive_joint_gains_gui_node')
        QWidget.__init__(self)

        self.init_ui()

        # ROS 2 Publishers and Subscribers
        self.pub = self.create_publisher(JointGains, 'joint_gains', 10)
        self.sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.sub = self.create_subscription(JointMotorControllerState, 'joint_motor_controller_state', self.joint_motor_controller_state_callback, 10)

        # Timer to check for received messages
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_message_status)
        self.timer.start(CHECK_MESSAGE_STATUS_INTERVAL)  
        self.joint_state_message_received = False
        self.motor_controller_message_received = False

    def init_ui(self):
        self.layout = QVBoxLayout()

        self.label = QLabel('Waiting for data...', self)
        self.layout.addWidget(self.label)

        self.button = QPushButton('Publish Joint Gains', self)
        self.button.clicked.connect(self.publish_message)
        self.layout.addWidget(self.button)

        self.setLayout(self.layout)
        self.setWindowTitle('Beautiful Interactive Joint Gains GUI')
        self.show()

    def joint_state_callback(self, msg):
        self.label.setText(f'Received Data: {msg.data}')
        self.joint_state_message_received = True

    def joint_motor_controller_state_callback(self, msg):
        self.label.setText(f'Received Data: {msg.data}')
        self.motor_controller_message_received = True

    def publish_message(self):
        msg = JointGains()
        msg.joint_name = "yomama_joint"  # Example joint name
        msg.proportional_gain = 1.0  # Set your actual values here
        msg.integral_gain = 0.1
        msg.derivative_gain = 0.01
        self.pub.publish(msg)

    def check_message_status(self):
        message = ''
        if not self.joint_state_message_received:
            message += 'No joint state data received.\n'
        if not self.motor_controller_message_received:
            message += 'No motor controller state data received.'
        self.label.setText(message if message else 'All data received.')
        

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    interactive_joint_gains_gui_node = InteractiveJointGainsGuiNode()
    sys.exit(app.exec_())
    interactive_joint_gains_gui_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

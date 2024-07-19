import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QLineEdit
from PyQt5.QtCore import QTimer, Qt

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from march_shared_msgs.msg import JointGains, JointMotorControllerState
from std_msgs.msg import Float64MultiArray

CHECK_MESSAGE_STATUS_INTERVAL = 5000  # milliseconds

class InteractiveJointGainsGuiNode(Node, QWidget):
    def __init__(self):
        Node.__init__(self, 'interactive_joint_gains_gui_node')
        QWidget.__init__(self)

        self.proportional_gain = 1.0
        self.integral_gain = 0.1
        self.derivative_gain = 0.01
        self.temp_proportional_gain = self.proportional_gain
        self.temp_integral_gain = self.integral_gain
        self.temp_derivative_gain = self.derivative_gain
        self.gains_changed = False
        self.is_publish_button_pressed = False

        self.init_ui()

        # ROS 2 Publishers and Subscribers
        self.pub = self.create_publisher(JointGains, 'joint_gains', 10)
        self.sub_joint_state = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.sub_joint_position_commands = self.create_subscription(Float64MultiArray, '/march_joint_position_controller/commands', self.joint_position_commands_callback, 10)
        self.sub_motor_controller_state = self.create_subscription(JointMotorControllerState, 'joint_motor_controller_state', self.joint_motor_controller_state_callback, 10)

        # Timer to check for received messages
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_message_status)
        self.timer.start(CHECK_MESSAGE_STATUS_INTERVAL)
        self.joint_state_message_received = False
        self.motor_controller_message_received = False
        self.joint_position_commands_received = False

    def init_ui(self):
        self.layout = QVBoxLayout()

        self.label = QLabel('Waiting for data...', self)
        self.layout.addWidget(self.label)

        self.publish_button = QPushButton('Publish New Gains', self)
        self.publish_button.clicked.connect(self.publish_new_gains)
        self.layout.addWidget(self.publish_button)

        # Proportional Gain Label
        self.pg_label = QLabel(f'Current proportional gain: {self.proportional_gain}', self)
        self.layout.addWidget(self.pg_label)

        # Integral Gain Label
        self.ig_label = QLabel(f'Current integral gain: {self.integral_gain}', self)
        self.layout.addWidget(self.ig_label)

        # Derivative Gain Label
        self.dg_label = QLabel(f'Current derivative gain: {self.derivative_gain}', self)
        self.layout.addWidget(self.dg_label)

        # Proportional Gain Input Field
        self.pg_input = QLineEdit(str(self.proportional_gain))
        self.pg_input.textChanged.connect(self.update_proportional_gain)
        self.layout.addWidget(self.pg_input)

        # Integral Gain Input Field
        self.ig_input = QLineEdit(str(self.integral_gain))
        self.ig_input.textChanged.connect(self.update_integral_gain)
        self.layout.addWidget(self.ig_input)

        # Derivative Gain Input Field
        self.dg_input = QLineEdit(str(self.derivative_gain))
        self.dg_input.textChanged.connect(self.update_derivative_gain)
        self.layout.addWidget(self.dg_input)

        self.pg_input.textChanged.connect(self.on_gain_changed)
        self.ig_input.textChanged.connect(self.on_gain_changed)
        self.dg_input.textChanged.connect(self.on_gain_changed)

        self.setLayout(self.layout)
        self.setWindowTitle('Beautiful Interactive Joint Gains GUI')
        self.show()

    def update_proportional_gain(self, value):
        try:
            self.temp_proportional_gain = float(value)
        except ValueError:
            self.pg_label.setText('Invalid input for Proportional Gain')

    def update_integral_gain(self, value):
        try:
            self.temp_integral_gain = float(value)
        except ValueError:
            self.ig_label.setText('Invalid input for Integral Gain')

    def update_derivative_gain(self, value):
        try:
            self.temp_derivative_gain = float(value)
        except ValueError:
            self.dg_label.setText('Invalid input for Derivative Gain')

    def joint_state_callback(self, msg):
        self.label.setText(f'Received Joint State Data')
        self.joint_state_message_received = True
        if self.joint_state_message_received:
            self.all_joint_positions = self.get_all_joint_positions(msg)

    def joint_motor_controller_state_callback(self, msg):
        self.label.setText(f'Received Motor Controller State Data')
        self.motor_controller_message_received = True
        if self.motor_controller_message_received:
            self.joint_currents = self.get_all_joint_currents(msg)

    def joint_position_commands_callback(self, msg):
        self.label.setText(f'Received Joint Position Commands Data')
        self.joint_position_commands_received = True
        if self.joint_position_commands_received:
            self.joint_position_commands = self.get_all_joint_position_commands(msg)

    def get_all_joint_positions(self, msg):
        joint_position_tuples = []
        self.all_joint_names = []
        for i in range(len(msg.name)):
            if 'ie' not in msg.name[i]:
                self.all_joint_names.append(msg.name[i])
                joint_position_tuples.append((msg.name[i], msg.position[i]))
        return joint_position_tuples

    def get_all_joint_currents(self, msg):
        joint_current_tuples = [] 
        for i in range(len(msg.joint_name)):
            if 'ie' not in msg.joint_name[i]:
                joint_current_tuples.append((msg.joint_name[i], msg.motor_current[i]))
        return joint_current_tuples
    
    def get_all_joint_position_commands(self, msg):
        joint_position_commands = []
        # Assuming that the joint names are in the same order as the joint position commands
        for i in range(len(msg.data)):
            joint_position_commands.append(self.all_joint_names[i], msg.data[i])
        return joint_position_commands

    def publish_new_gains(self):
        self.is_publish_button_pressed = True
        self.publish_gains()

        if self.gains_changed and self.is_publish_button_pressed:
            # Update the actual gain attributes
            self.proportional_gain = self.temp_proportional_gain
            self.integral_gain = self.temp_integral_gain
            self.derivative_gain = self.temp_derivative_gain

            # Update the labels to reflect the new gains
            self.pg_label.setText(f'Proportional Gain: {self.proportional_gain:.2f}')
            self.ig_label.setText(f'Integral Gain: {self.integral_gain:.2f}')
            self.dg_label.setText(f'Derivative Gain: {self.derivative_gain:.2f}')

        self.gains_changed = False
        self.is_publish_button_pressed = False
        self.publish_button.setEnabled(self.gains_changed)

    def on_gain_changed(self):
        self.gains_changed = True
        self.publish_button.setEnabled(self.gains_changed)

    def publish_gains(self):
        msg = JointGains()
        msg.joint_name = 'yomama'
        msg.proportional_gain = self.temp_proportional_gain
        msg.integral_gain = self.temp_integral_gain
        msg.derivative_gain = self.temp_derivative_gain
        self.pub.publish(msg)

    def check_message_status(self):
        message = ''
        if not self.joint_state_message_received:
            message += 'No joint state data received.\n'
        if not self.motor_controller_message_received:
            message += 'No motor controller state data received.\n'
        if not self.joint_position_commands_received:
            message += 'No joint position commands data received'
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

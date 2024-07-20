import sys
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QPushButton, QLabel, QLineEdit, QFrame, QVBoxLayout
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
        self.layout = QGridLayout()
        self.layout.setSpacing(0)  # Remove space between cells
        # self.layout.setContentsMargins(0, 0, 0, 0)  # Remove margins

        def add_grid_cell(widget, row, col, center=True, side_margins=0,top_margins=3, color=None):
            frame = QFrame()
            inner_layout = QVBoxLayout()
            inner_layout.addWidget(widget)
            if center:
                inner_layout.setAlignment(Qt.AlignCenter)  # Center the text within the cell
            if color:
                frame.setStyleSheet(f'background-color: {color};')
            inner_layout.setContentsMargins(side_margins,top_margins, side_margins,top_margins) 
            frame.setLayout(inner_layout)
            frame.setFrameShape(QFrame.Box)
            self.layout.addWidget(frame, row, col)
        
        # Helper function to create bold labels
        def create_bold_label(text):
            label = QLabel(text)
            label.setStyleSheet("font-weight: bold; color: white;")
            return label

        # Column Headers
        add_grid_cell(create_bold_label('Gains'), 0, 0,color='#26a69a')
        add_grid_cell(create_bold_label('Proportional'), 0, 1,color='#26a69a')
        add_grid_cell(create_bold_label('Integral'), 0, 2,color='#26a69a')
        add_grid_cell(create_bold_label('Derivative'), 0, 3,color='#26a69a')

        # Current Gains
        add_grid_cell(QLabel('Current Gains'), 1, 0, center=False, side_margins=4, color='white')
        self.pg_label = QLabel(f'{self.proportional_gain:.2f}')
        add_grid_cell(self.pg_label, 1, 1, color='white')
        self.ig_label = QLabel(f'{self.integral_gain:.2f}')
        add_grid_cell(self.ig_label, 1, 2, color='white')
        self.dg_label = QLabel(f'{self.derivative_gain:.2f}')
        add_grid_cell(self.dg_label, 1, 3, color='white')

        # New Gains Input Fields
        add_grid_cell(QLabel('New Gains'), 2, 0, center=False, side_margins=4,top_margins=0,color='#ddf2f0')
        self.pg_input = QLineEdit(str(self.proportional_gain))
        self.pg_input.setAlignment(Qt.AlignCenter)
        self.pg_input.textChanged.connect(self.update_proportional_gain)
        add_grid_cell(self.pg_input, 2, 1,top_margins=0,color='#ddf2f0')

        self.ig_input = QLineEdit(str(self.integral_gain))
        self.ig_input.setAlignment(Qt.AlignCenter)  
        self.ig_input.textChanged.connect(self.update_integral_gain)
        add_grid_cell(self.ig_input, 2, 2,top_margins=0,color='#ddf2f0')

        self.dg_input = QLineEdit(str(self.derivative_gain))
        self.dg_input.setAlignment(Qt.AlignCenter)
        self.dg_input.textChanged.connect(self.update_derivative_gain)
        add_grid_cell(self.dg_input, 2, 3,top_margins=0,color='#ddf2f0')

        self.pg_input.textChanged.connect(self.on_gain_changed)
        self.ig_input.textChanged.connect(self.on_gain_changed)
        self.dg_input.textChanged.connect(self.on_gain_changed)

        # Publish Button
        self.publish_button = QPushButton('Publish New Gains', self)
        self.publish_button.clicked.connect(self.publish_new_gains)
        self.layout.addWidget(self.publish_button, 3, 0, 1, 4,)

        # Status Label
        self.label = QLabel('Waiting for data...', self)
        self.layout.addWidget(self.label, 4, 0, 1, 4)

        self.setLayout(self.layout)
        self.setWindowTitle('Interactive Joint Gains GUI')
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
            self.joint_position_commands = msg.data

    def get_all_joint_currents(self, msg):
        joint_current_tuples = []
        for joint_name, current in zip(msg.joint_name, msg.current):
            if 'ie' not in joint_name:
                joint_current_tuples.append((joint_name, current))
        return joint_current_tuples

    def publish_new_gains(self):
        self.is_publish_button_pressed = True
        self.publish_gains()

        if self.gains_changed and self.is_publish_button_pressed:
            # Update the actual gain attributes
            self.proportional_gain = self.temp_proportional_gain
            self.integral_gain = self.temp_integral_gain
            self.derivative_gain = self.temp_derivative_gain

            # Update the labels to reflect the new gains
            self.pg_label.setText(f'{self.proportional_gain:.2f}')
            self.ig_label.setText(f'{self.integral_gain:.2f}')
            self.dg_label.setText(f'{self.derivative_gain:.2f}')

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

    def get_all_joint_positions(self, msg):
        joint_position_tuples = []
        for i in range(len(msg.name)):
            if 'ie' not in msg.name[i]:
                joint_position_tuples.append((msg.name[i], msg.position[i]))
        return joint_position_tuples

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

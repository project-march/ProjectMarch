from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg
from flask import jsonify, make_response


class MpcListener(Node):
    def __init__(self):
        super().__init__('mpc_listener')
        self.subscription = self.create_subscription(MpcMsg, '/march/mpc', self.listener_callback, 1)

        self.number_of_joints = 0

        self.new_data_position = []
        self.new_data_velocity = []

        self.response_body = {}

    # Set all data
    def listener_callback(self, msg):
        self.number_of_joints = len(msg.joint)

        # Current position and velocity
        for joint_number in range(self.number_of_joints):
            self.new_data_position = msg.joint[joint_number].estimation.states[0].array
            self.new_data_velocity = msg.joint[joint_number].estimation.states[1].array

        # Estimation
        print(type(msg.joint[joint_number].estimation.states[0].array))

    # @app.route("/")
    def stream_measurement(self):
        for joint_number in range(self.number_of_joints):
            self.response_body[f'joint_{joint_number}_position'] = self.new_data_position[joint_number]
            self.response_body[f'joint_{joint_number}_velocity'] = self.new_data_velocity[joint_number]

        # Bokeh requires JSON format
        response = make_response(jsonify(self.response_body), 200)
        return response

    def stream_estimation(self):
        pass

from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg
from flask import jsonify, make_response
import numpy as np


class MpcListener(Node):
    def __init__(self):
        super().__init__('mpc_listener')
        self.subscription = self.create_subscription(MpcMsg, '/march/mpc', self.listener_callback, 1)

        self.number_of_joints = 0

        self.new_data_position = []
        self.new_data_velocity = []

        self.new_estimation_position = np.empty([0, 0], dtype=float)
        self.new_estimation_velocity = np.empty([0, 0], dtype=float)
        self.new_estimation_input = np.empty([0, 0], dtype=float)

        self.response_body_measurement = {}
        self.response_body_estimation = {}

    # Set all data
    def listener_callback(self, msg):
        self.number_of_joints = len(msg.joint)

        # Current position and velocity
        for joint_number in range(self.number_of_joints):
            self.new_data_position = msg.joint[joint_number].estimation.states[0].array
            self.new_data_velocity = msg.joint[joint_number].estimation.states[1].array

            # Estimation
            self.new_estimation_position = msg.joint[joint_number].estimation.states[0].array[1:-1]
            self.new_estimation_velocity = msg.joint[joint_number].estimation.states[1].array[1:-1]
            self.new_estimation_input = msg.joint[joint_number].estimation.inputs[0].array

    # @app.route("/")
    def stream_measurement(self):
        for joint_number in range(self.number_of_joints):
            self.response_body_measurement[f'joint_{joint_number}_position'] = self.new_data_position[joint_number]
            self.response_body_measurement[f'joint_{joint_number}_velocity'] = self.new_data_velocity[joint_number]

        return make_response(jsonify(self.response_body_measurement), 200)

    def stream_estimation(self):
        for joint_number in range(self.number_of_joints):
            self.response_body_estimation[f'joint_{joint_number}_estimation_position'] \
                = self.new_estimation_position.tolist()
            self.response_body_estimation[f'joint_{joint_number}_estimation_velocity'] \
                = self.new_estimation_velocity.tolist()
            self.response_body_estimation[f'joint_{joint_number}_estimation_input'] \
                = self.new_estimation_input.tolist()

        return make_response((jsonify(self.response_body_estimation)))

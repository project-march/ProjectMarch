from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg
from flask import jsonify, make_response, request, Response
from itertools import chain
import numpy as np

MAX_AGE = str(21600)


class MpcListener(Node):
    def __init__(self):
        super().__init__("mpc_listener")
        self.subscription = self.create_subscription(
            MpcMsg, "/march/mpc", self.mpc_topic_listener_callback, 1
        )

        self.number_of_joints = 0
        self.future_time_steps = 0  # TODO: read from message after MPC MR
        self.sampling_time = 0.02  # TODO: read from message after MPC MR

        self.new_measurement_position = []
        self.new_measurement_velocity = []
        self.new_time = 0

        self.new_estimation_position = np.empty([0, 0], dtype=float)
        self.new_estimation_velocity = np.empty([0, 0], dtype=float)
        self.new_estimation_input = np.empty([0, 0], dtype=float)

        self.response_body_measurement = {}
        self.response_body_estimation = {}

    # Set all data
    def mpc_topic_listener_callback(self, msg):
        """
        Updates all arrays that are streamed to the host. Array indices are chosen such that the can be plotted
        immediately. states[0] contains position, states[1] the velocity, as defined in march_acado_mpc package.
        The current state is at array[], and the estimation are all the following values.
        :param msg: march_shared_msgs.MpcMsg
        :return:
        """

        self.number_of_joints = len(msg.joint)
        self.future_time_steps = len(msg.joint[0].estimation.states[0].array[1:])

        if not self.new_estimation_position.any():
            self.set_lengths()

        # Current position and velocity
        for joint_number in range(self.number_of_joints):
            self.new_measurement_position.insert(joint_number, msg.joint[joint_number].estimation.states[0].array[0])
            self.new_measurement_velocity.insert(joint_number, msg.joint[joint_number].estimation.states[1].array[0])

            # Estimation
            self.new_estimation_position[joint_number, :] = (
                msg.joint[joint_number].estimation.states[0].array[1:]
            )
            self.new_estimation_velocity[joint_number, :] = (
                msg.joint[joint_number].estimation.states[1].array[1:]
            )
            self.new_estimation_input[joint_number, :] = (
                msg.joint[joint_number].estimation.inputs[0].array
            )

        # Get time
        self.new_time = msg.header.stamp.sec + msg.header.stamp.nanosec * (10**-9)

        # print(self.new_estimation_position[0, :].tolist())
        # print(chain.from_iterable(self.new_estimation_position.tolist()))

    def set_lengths(self):
        self.new_estimation_position = np.empty(shape=(self.number_of_joints, self.future_time_steps), dtype=float)
        self.new_estimation_velocity = np.empty(shape=(self.number_of_joints, self.future_time_steps), dtype=float)
        self.new_estimation_input = np.empty(shape=(self.number_of_joints, self.future_time_steps), dtype=float)

    @staticmethod
    def set_headers(response: Response) -> Response:
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = "GET, OPTIONS, POST"
        response.headers['Access-Control-Max-Age'] = str(MAX_AGE)
        requested_headers = request.headers.get('Access-Control-Request-Headers')
        if requested_headers:
            response.headers['Access-Control-Allow-Headers'] = requested_headers

        return response

    # @app.route("/measurement")
    def stream_measurement(self):
        for joint_number in range(self.number_of_joints):
            self.response_body_measurement[
                f"joint_{joint_number}_position"
            ] = self.new_measurement_position[joint_number]
            self.response_body_measurement[
                f"joint_{joint_number}_velocity"
            ] = self.new_measurement_velocity[joint_number]
            self.response_body_measurement["time_stamp"] = self.new_time

        response = make_response(jsonify(self.response_body_measurement))
        response = self.set_headers(response)
        return response

    # @app.route("/estimation")
    def stream_estimation(self):
        for joint_number in range(self.number_of_joints):
            self.response_body_estimation[
                f"joint_{joint_number}_estimation_position"
            ] = self.new_estimation_position[joint_number, :].tolist()
            self.response_body_estimation[
                f"joint_{joint_number}_estimation_velocity"
            ] = self.new_estimation_velocity[joint_number, :].tolist()
            self.response_body_estimation[
                f"joint_{joint_number}_estimation_input"
            ] = self.new_estimation_input[joint_number, :].tolist()
            self.response_body_estimation["time"] = np.linspace(start=self.new_time,
                                                                stop=self.new_time + self.future_time_steps * self.sampling_time,
                                                                num=self.future_time_steps,
                                                                endpoint=False).tolist()

            response = make_response(jsonify(self.response_body_estimation))
            response = self.set_headers(response)

            return response


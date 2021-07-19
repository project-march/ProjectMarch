from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg
from flask import jsonify, make_response, request, Response
import numpy as np

MAX_AGE = str(21600)
NANO = 10 ** -9


class MpcListener(Node):
    def __init__(self):
        super().__init__("mpc_listener")
        self.subscription = self.create_subscription(
            MpcMsg, "/march/mpc", self.mpc_topic_listener_callback, 1
        )

        self.number_of_joints = 0
        self.future_time_steps = 0
        self.sampling_time = 0.02

        # Indexing over joint number
        self.new_measurement_position = []
        self.new_measurement_velocity = []
        self.last_input = []
        self.last_reference_position = []
        self.last_reference_velocity = []
        self.last_reference_input = []

        self.new_time = 0

        # Indexing over joint number and time
        self.new_estimation_position = np.empty([0, 0], dtype=float)
        self.new_estimation_velocity = np.empty([0, 0], dtype=float)
        self.new_estimation_input = np.empty([0, 0], dtype=float)
        self.future_reference_position = np.empty([0, 0], dtype=float)
        self.future_reference_velocity = np.empty([0, 0], dtype=float)
        self.future_reference_input = np.empty([0, 0], dtype=float)

        self.response_body_measurement = {}
        self.response_body_estimation = {}

    # Reset data
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
        joint0_positions = msg.joint[0].estimation.states[0]
        self.future_time_steps = len(joint0_positions.array[1:])

        if not self.new_estimation_position.any():
            self.set_lengths()

        for joint_number in range(self.number_of_joints):
            # Set old estimated input as last applied input
            self.last_input[joint_number] = self.new_estimation_input[joint_number, 0]

            # Current position and velocity
            joint_pos = msg.joint[joint_number].estimation.states[0]
            joint_vel = msg.joint[joint_number].estimation.states[1]
            joint_input = msg.joint[joint_number].estimation.inputs[0]
            joint_reference = msg.joint[joint_number].reference

            self.new_measurement_position[joint_number] = joint_pos.array[0]
            self.new_measurement_velocity[joint_number] = joint_vel.array[0]

            # And its reference
            self.last_reference_position[joint_number] = self.future_reference_position[joint_number, 0]
            self.last_reference_velocity[joint_number] = self.future_reference_position[joint_number, 0]
            self.last_reference_input[joint_number] = self.future_reference_position[joint_number, 0]

            # Estimation
            self.new_estimation_position[joint_number, :] = joint_pos.array[1:]
            self.new_estimation_velocity[joint_number, :] = joint_vel.array[1:]
            self.new_estimation_input[joint_number, :] = joint_input.array

            self.future_reference_position[joint_number, :] = joint_reference.states[0].array[1:]
            self.future_reference_velocity[joint_number, :] = joint_reference.states[1].array[1:]
            self.future_reference_input[joint_number, :] = joint_reference.inputs[0].array

        # Get time
        self.new_time = msg.header.stamp.sec + msg.header.stamp.nanosec * NANO

    def set_lengths(self):
        self.new_measurement_position = [None] * self.number_of_joints
        self.new_measurement_velocity = [None] * self.number_of_joints
        self.last_input = [None] * self.number_of_joints
        self.last_reference_position = [None] * self.number_of_joints
        self.last_reference_velocity = [None] * self.number_of_joints
        self.last_reference_input = [None] * self.number_of_joints

        self.new_estimation_position = np.empty(
            shape=(self.number_of_joints, self.future_time_steps), dtype=float
        )
        self.new_estimation_velocity = np.empty(
            shape=(self.number_of_joints, self.future_time_steps), dtype=float
        )
        self.new_estimation_input = np.empty(
            shape=(self.number_of_joints, self.future_time_steps), dtype=float
        )
        self.future_reference_position = np.empty(
            shape=(self.number_of_joints, self.future_time_steps), dtype=float
        )
        self.future_reference_velocity = np.empty(
            shape=(self.number_of_joints, self.future_time_steps), dtype=float
        )
        self.future_reference_input = np.empty(
            shape=(self.number_of_joints, self.future_time_steps), dtype=float
        )

    @staticmethod
    def set_headers(response: Response) -> Response:
        response.headers["Access-Control-Allow-Origin"] = "*"
        response.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS, POST"
        response.headers["Access-Control-Max-Age"] = str(MAX_AGE)
        requested_headers = request.headers.get("Access-Control-Request-Headers")
        if requested_headers:
            response.headers["Access-Control-Allow-Headers"] = requested_headers

        return response

    # @app.route("/measurement")
    def stream_measurement(self):
        for joint_number in range(self.number_of_joints):
            self.response_body_measurement[f"joint_{joint_number}_position"] = [
                self.new_measurement_position[joint_number],
            ]
            self.response_body_measurement[f"joint_{joint_number}_reference_position"] = [
                self.last_reference_position[joint_number]
            ]

            self.response_body_measurement[f"joint_{joint_number}_velocity"] = [
                self.new_measurement_velocity[joint_number],
            ]
            self.response_body_measurement[f"joint_{joint_number}_reference_velocity"] = [
                self.last_reference_velocity[joint_number]
            ]

            self.response_body_measurement[f"joint_{joint_number}_input"] = [
                self.last_input[joint_number],
            ]
            self.response_body_measurement[f"joint_{joint_number}_reference_input"] = [
                self.last_reference_input[joint_number]
            ]

            self.response_body_measurement["time"] = [self.new_time]

        response = make_response(jsonify(self.response_body_measurement))
        return self.set_headers(response)

    # @app.route("/estimation")
    def stream_estimation(self):
        for joint_number in range(self.number_of_joints):
            self.response_body_estimation[
                f"joint_{joint_number}_estimation_position"
            ] = self.new_estimation_position[joint_number, :].tolist()
            self.response_body_estimation[
                f"joint_{joint_number}_reference_position"
            ] = self.future_reference_position[joint_number, :].tolist()
            self.response_body_estimation[
                f"joint_{joint_number}_estimation_velocity"
            ] = self.new_estimation_velocity[joint_number, :].tolist()
            self.response_body_estimation[
                f"joint_{joint_number}_reference_velocity"
            ] = self.future_reference_velocity[joint_number, :].tolist()
            self.response_body_estimation[
                f"joint_{joint_number}_estimation_input"
            ] = self.new_estimation_input[joint_number, :].tolist()
            self.response_body_estimation[
                f"joint_{joint_number}_reference_input"
            ] = self.future_reference_input[joint_number, :].tolist()

            self.response_body_estimation["time"] = np.linspace(
                start=self.new_time,
                stop=self.new_time + self.future_time_steps * self.sampling_time,
                num=self.future_time_steps,
                endpoint=False,
            ).tolist()

        response = make_response(jsonify(self.response_body_estimation))
        return self.set_headers(response)

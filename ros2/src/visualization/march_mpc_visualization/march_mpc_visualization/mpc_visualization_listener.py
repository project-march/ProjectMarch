import rclpy
from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg


from flask import jsonify, make_response
#import numpy as np
#from march_mpc_visualization import app

STREAM_PERIOD = 0.02


class MpcListener(Node):
    def __init__(self):
        super().__init__('mpc_listener')
        self.subscription = self.create_subscription(MpcMsg, '/march/mpc', self.listener_callback, 1)
        self.test = "test"
        #self.number_of_joints = 8

        self.new_data_position = 3.0

        # 0 - 100
        # Current actual -> append
        # Desired N -> replace
        # Calculated input N -> replace

        #self.create_timer(STREAM_PERIOD, self.stream)

    # Set all data
    def listener_callback(self, msg):
        self.new_data_position = msg.header.stamp.nanosec
        self.get_logger().info('I heard: "%f"' % self.new_data_position)

    def init_bokeh(self):
        pass

    # callback, of op timer
    # @app.route("/")
    def stream(self):
        response_body = {
            "x": 5,
            "y": 10
        }

        response_body["x"] = self.new_data_position
        response = make_response(jsonify(response_body), 200)
        return response

import rclpy
from rclpy.node import Node
from march_shared_msgs.msg import MpcMsg

from flask import jsonify, make_response
from march_mpc_visualization import app

STREAM_PERIOD = 0.02


class MpcListener(Node):
    def __init__(self):
        super().__init__('mpc_listener')
        self.subscription = self.create_subscription(MpcMsg, '/march/mpc', self.listener_callback, 1)

        #self.number_of_joints = 8

        #self.new_data_position = 0.0


        # 0 - 100
        # Current actual -> append
        # Desired N -> replace
        # Calculated input N -> replace

        #self.create_timer(STREAM_PERIOD, self.stream)

    # Set all data
    def listener_callback(self, msg):
        pass
        #self.new_data_position = msg.joint[0].estimation.states[0].array[0]
        #self.get_logger().info('I heard: "%f"' % self.new_data_position)

    def init_bokeh(self):
        pass

    # callback, of op timer
    @app.route("/json")
    def stream(self):
        response_body = {
            "x": 5,
            "y": 10
        }
        response = make_response(jsonify(response_body), 200)
        return "Thanks"


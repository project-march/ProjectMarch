import rclpy
from rclpy.node import Node
from mpc_visualization_listener import MpcListener


class MpcStreamer(node):
    def __init__(self, MpcListener):
        pass

    def init_bokeh(self):
        pass

    # callback, of op timer
    def stream(self, MpcListener):
        pass

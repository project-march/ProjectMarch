import rclpy
from rclpy.node import Node
from march_mpc_visualization.mpc_visualization_listener import MpcListener
from march_mpc_visualization.mpc_visualization_streamer import MpcStreamer



def main(args=None):
    """The main function used to start up the rqt note taker."""
    rclpy.init(args=args)

    #node = Node('march_mpc_visualization_node')
    mpc_listener = MpcListener()
    mpc_streamer = MpcStreamer(mpc_listener)

    try:
        rclpy.spin(mpc_listener)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

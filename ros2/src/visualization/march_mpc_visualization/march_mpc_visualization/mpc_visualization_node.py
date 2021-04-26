import rclpy
from march_mpc_visualization.mpc_visualization_listener import MpcListener
from march_mpc_visualization import app


def main(args=None):
    """The main function used to start up the rqt note taker."""
    rclpy.init(args=args)

    mpc_listener = MpcListener()
    app.run()

    try:
        rclpy.spin(mpc_listener)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

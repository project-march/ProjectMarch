import rclpy
from march_mpc_visualization.mpc_visualization_listener import MpcListener
from march_mpc_visualization import app
import threading


def main():
    rclpy.init()
    mpc_listener = MpcListener()

    # Add stream method to flask app
    app.add_url_rule(
        "/measurement", "stream_measurement", mpc_listener.stream_measurement
    )
    app.add_url_rule("/estimation", "stream_estimation", mpc_listener.stream_estimation)

    # ROS Node and Flask app need to run in seperatie thread

    flask_thread = threading.Thread(target=lambda: app.run(host="0.0.0.0"))

    # Daemon, so that it shuts down when main() finishes
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(mpc_listener)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

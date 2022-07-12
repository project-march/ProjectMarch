"""Author: Thijs Veen, MVI."""
import rclpy
from march_mpc_visualization.mpc_visualization_listener import MpcListener
from march_mpc_visualization import app
import threading

from contextlib import suppress


def main():
    """Starts the mpc listener node."""
    rclpy.init()
    mpc_listener = MpcListener()

    # Add stream method to flask app
    app.add_url_rule(
        rule="/measurement",
        endpoint="stream_measurement",
        view_func=mpc_listener.stream_measurement,
        methods=["GET", "OPTIONS", "POST"],
    )
    app.add_url_rule(
        rule="/estimation",
        view_func=mpc_listener.stream_estimation,
        methods=["GET", "OPTIONS", "POST"],
    )

    # We want to bind to all interfaces
    flask_thread = threading.Thread(target=lambda: app.run(host="0.0.0.0"))  # noqa: S104

    # Daemon, so that it shuts down when main() finishes
    flask_thread.daemon = True
    flask_thread.start()

    with suppress(KeyboardInterrupt):
        rclpy.spin(mpc_listener)

    rclpy.shutdown()

"""Author: Tuhin Das, MVII."""

from .connection_manager import ConnectionManager
from .wireless_ipd_controller import WirelessInputDeviceController
import rclpy
import threading
import sys
from rclpy.executors import MultiThreadedExecutor

NODE_NAME = "march_wireless_ipd_node"
PORT = 4000


def sys_exit(*_):
    """Exit cleanly."""
    sys.exit(0)


def main():
    """Initialize wireless IPD node."""
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    node.declare_parameter("ip_address")
    logger = node.get_logger()
    controller = WirelessInputDeviceController(node, logger)
    ip = node.get_parameter("ip_address").get_parameter_value().string_value
    manager = ConnectionManager(ip, PORT, controller, node, logger)
    executor = MultiThreadedExecutor()

    def spin_node():
        rclpy.spin(node, executor)

    # Spinning is done from a new thread, because manager.establish_connection() is blocking
    spin_thread = threading.Thread(target=spin_node)
    spin_thread.daemon = True
    spin_thread.start()

    manager.establish_connection()


if __name__ == "__main__":
    main()

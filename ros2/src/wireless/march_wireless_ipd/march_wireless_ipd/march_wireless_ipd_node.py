"""Author: Tuhin Das, MVII."""

from .connection_manager import ConnectionManager
from .input_device_controller import WirelessInputDeviceController
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from march_utility.utilities.logger import Logger
import threading
import signal
import sys
from contextlib import suppress


class WirelessIPDNode(Node):
    """Node that runs the wireless IPD."""

    def __init__(self):
        super().__init__("wireless_ipd_node")


def sys_exit(*_):
    """Exit cleanly."""
    sys.exit(0)


def main():
    """Initialize wireless IPD node."""
    ip = "192.168.0.100"
    rclpy.init()

    node = WirelessIPDNode()
    executor = MultiThreadedExecutor()
    logger = Logger(node, "WirelessIPDNode")
    controller = WirelessInputDeviceController(node, logger)
    manager = ConnectionManager(ip, 4000, controller, node, logger)
    thr = threading.Thread(target=manager.establish_connection)
    thr.start()

    signal.signal(signal.SIGTERM, sys_exit)

    with suppress(KeyboardInterrupt):
        rclpy.spin(node, executor)

    rclpy.shutdown()

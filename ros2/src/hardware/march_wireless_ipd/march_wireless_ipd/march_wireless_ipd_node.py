"""Author: Tuhin Das, MVII."""

from .connection_manager import ConnectionManager
from .wireless_ipd_controller import WirelessInputDeviceController
import rclpy
from march_utility.utilities.logger import Logger
import threading
import signal
import sys
from contextlib import suppress
from netifaces import interfaces, ifaddresses, AF_INET
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


def sys_exit(*_):
    """Exit cleanly."""
    sys.exit(0)


def main():
    """Initialize wireless IPD node."""

    ip = "192.168.0.100"
    for iface in interfaces():
        iface_details = ifaddresses(iface)
        if AF_INET in iface_details:
            interface_info = iface_details[AF_INET][0]
            if "addr" in interface_info:
                address = interface_info["addr"]
                if address[0:3] == "192":
                    ip = address

    rclpy.init()

    node = rclpy.create_node("march_wireless_ipd_node")
    logger = Logger(node, "march_wireless_ipd_node")
    controller = WirelessInputDeviceController(node, logger)
    manager = ConnectionManager(ip, 4000, controller, node, logger)
    executor = MultiThreadedExecutor()

    def spin_thread():
        rclpy.spin(node, executor)

    thr = threading.Thread(target=spin_thread)
    thr.daemon = True
    thr.start()

    manager.establish_connection()

    signal.signal(signal.SIGTERM, sys_exit)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

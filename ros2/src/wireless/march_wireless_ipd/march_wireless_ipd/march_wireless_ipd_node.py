from .connection_manager import ConnectionManager
from .input_device_controller import InputDeviceController
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
from netifaces import interfaces, ifaddresses, AF_INET
import signal
import sys
from contextlib import suppress

class WirelessIPDNode(Node):

    def __init__(self):
        super().__init__('wireless_ipd_node')

def sys_exit(*_):
    sys.exit(0)

def main():
    
    for iface in interfaces():
        iface_details = ifaddresses(iface)
        if AF_INET in iface_details:
            interface_info = iface_details[AF_INET][0]
            if 'addr' in interface_info:
                address = interface_info['addr']
                if address[0:3] == '192':
                    ip = address

    rclpy.init()

    node = WirelessIPDNode()
    executor = MultiThreadedExecutor()
    controller = InputDeviceController(node)
    manager = ConnectionManager(ip, 4000, controller, node)
    thr = threading.Thread(target=manager.establish_connection)
    thr.start()

    signal.signal(signal.SIGTERM, sys_exit)

    with suppress(KeyboardInterrupt):
        rclpy.spin(node, executor)

    rclpy.shutdown()


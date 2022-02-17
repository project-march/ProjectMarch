from .connection_manager import ConnectionManager
from .input_device_controller import InputDeviceController
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import socket


class WirelessIPDNode(Node):

    def __init__(self):
        super().__init__('wireless_ipd_node')

def main():
    
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    ip = s.getsockname()[0]
    s.close()

    rclpy.init()

    node = WirelessIPDNode()
    executor = MultiThreadedExecutor()
    controller = InputDeviceController(node)
    manager = ConnectionManager(ip, 4000, controller, node)
    thr = threading.Thread(target=manager.establish_connection)
    thr.start()

    rclpy.spin(node, executor)
    rclpy.shutdown()


from .connection_manager import ConnectionManager
from .input_device_controller import InputDeviceController
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading


class WirelessIPDNode(Node):

    def __init__(self):
        super().__init__('wireless_ipd_node')

def main():
    rclpy.init()

    node = WirelessIPDNode()
    executor = MultiThreadedExecutor()
    controller = InputDeviceController(node)
    manager = ConnectionManager('192.168.148.240', 4000, controller, node)
    thr = threading.Thread(target=manager.establish_connection)
    thr.start()

    rclpy.spin(node, executor)
    rclpy.shutdown()


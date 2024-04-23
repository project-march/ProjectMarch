"""Author: Katja Schmal, MVI & Marco Bak, MVIII & Andrew Hutani, MIX"""
import getpass
import socket

from march_shared_msgs.msg import Alive
from march_shared_msgs.srv import GetExoModeArray
from rclpy.node import Node
import rclpy
import threading

from march_rqt_input_device.input_device import IPD

class InputDeviceController:
    """
    The controller for the input device, uses the node provided in the rqt context.
    """

    ID_FORMAT = "rqt@{machine}@{user}ros2"

    def __init__(self, node: Node):
        self._ipd = IPD()
        self._node = node
        self._view = None
        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())

        self._requested_mode = GetExoModeArray.Request()
        self._available_modes_future = None

        self._get_exo_mode_array_client = self._node.create_client(GetExoModeArray, 'get_exo_mode_array')

        while not self._get_exo_mode_array_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn("Waiting for service get_exo_mode_array to become available...")

        self._node.get_logger().info("Connected to service get_exo_mode_array")

        self._alive_pub = self._node.create_publisher(Alive, "/march/input_device/alive", 10)
        self._alive_timer = self._node.create_timer(timer_period_sec=0.1,
                                                    callback=self.alive_callback,
                                                    clock=self._node.get_clock())
        

        

    def __del__(self):
        """Deconstructer, that shutsdown the publishers and resets the timers."""
        self._node.destroy_publisher(self._instruction_gait_pub)
        self._node.destroy_publisher(self._error_pub)
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_timer.join()
            self._alive_pub.unregister()

    def set_view(self, view):
        self._view = view

    @property
    def node(self):
        """Define the node."""
        return self._node

    def publish_mode(self, mode: int) -> None:
        self._requested_mode.desired_mode.mode = mode
        self._available_modes_future = self._get_exo_mode_array_client.call_async(self._requested_mode)
        self._view.update_possible_modes()

    def store_available_modes(self, future) -> None:
        if future.result() is None:
            self._node.get_logger().warn("No available modes received")
            return
        
        available_modes = future.result().mode_array.modes
        mode_list = [exo_mode.mode for exo_mode in available_modes]
        self._ipd.set_available_modes(set(mode_list))
        self.update_view_buttons()

    def update_view_buttons(self):
        available_modes = self._ipd.get_available_modes()
        self._view.update_buttons(available_modes)
    
    def alive_callback(self) -> None:
        """Callback to send out an alive message."""
        msg = Alive(stamp=self._node.get_clock().now().to_msg(), id=self._id)
        self._alive_pub.publish(msg)

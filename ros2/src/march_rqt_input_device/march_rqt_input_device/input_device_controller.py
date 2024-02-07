"""Author: Katja Schmal, MVI & Marco Bak, MVIII & Andrew Hutani, MIX"""
import getpass
import socket

from march_shared_msgs.msg import Alive
from march_shared_msgs.srv import GetExoModeArray
from rclpy.node import Node
import rclpy

class IPD:
    def __init__(self):
        self._current_mode = None
        self._available_modes = set()

    def get_current_mode(self):
        return self._current_mode

    def get_available_modes(self):
        return self._available_modes

    def ask_new_mode(self):
        # Implement the logic to ask for a new mode here
        pass

    def set_current_mode(self, current_mode):
        self._current_mode = current_mode

    def set_available_modes(self, available_modes):
        self._available_modes = available_modes

class InputDeviceController:
    """
    The controller for the input device, uses the node provided in the rqt context.
    """

    ID_FORMAT = "rqt@{machine}@{user}ros2"

    def __init__(self, node: Node):
        self._ipd = IPD()
        self._node = node
        self._view = None

        self._get_exo_mode_array_client = self._node.create_client(GetExoModeArray, 'get_exo_mode_array')

        while not self._get_exo_mode_array_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn("Waiting for service get_exo_mode_array to become available...")

        self._node.get_logger().info("Connected to service get_exo_mode_array")

        self._alive_pub = self._node.create_publisher(Alive, "/march/input_device/alive", 10)
        self._alive_timer = self._node.create_timer(timer_period_sec=0.1,
                                                    callback=self.alive_callback,
                                                    clock=self._node.get_clock())
        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())

        

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

    def publish_home_stand(self) -> None:
        self.publish_mode(1)
    
    def publish_normal_walk(self) -> None:
        self.publish_mode(2)

    def publish_sit(self) -> None:
        self.publish_mode(0)

    def publish_sideways_walk(self) -> None:
        self.publish_mode(5)

    def publish_ascending_walk(self) -> None:
        self.publish_mode(8)

    def publish_descending_walk(self) -> None:
        self.publish_mode(9)
    
    def publish_large_walk(self) -> None:
        self.publish_mode(10)

    def publish_small_walk(self) -> None:
        self.publish_mode(7)

    
    def publish_variable_walk(self) -> None:
        self.publish_mode(10)

    def publish_mode(self, mode: int) -> None:
        self._ipd.set_current_mode(mode)
        request = GetExoModeArray.Request()
        request.desired_mode.mode = mode

        future = self._get_exo_mode_array_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)

        if future.result() is not None:
            self.store_available_modes(future)
        else:
            self._node.get_logger().error('Exception while calling service: %r' % future.exception()) 

    def store_available_modes(self, future) -> None:
        rclpy.spin_until_future_complete(self._node, future)
        if future.done():
            available_modes = future.result().mode_array.modes
            mode_list = [exo_mode.mode for exo_mode in available_modes]
            self._ipd.set_available_modes(set(mode_list))
            self.update_view_buttons()
        else:
            self._node.get_logger().error("Failed to call service GetExoModeArray")

    def update_view_buttons(self):
        available_modes = self._ipd.get_available_modes()
        self._view.update_buttons(available_modes)
    
    def alive_callback(self) -> None:
        """Callback to send out an alive message."""
        msg = Alive(stamp=self._node.get_clock().now().to_msg(), id=self._id)
        self._alive_pub.publish(msg)

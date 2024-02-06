"""Author: Katja Schmal, MVI & Marco Bak, MVIII."""
import getpass
import socket

from march_shared_msgs.srv import GetExoModeArray
from rclpy.node import Node
import rclpy


class InputDeviceController:
    """
    The controller for the input device, uses the node provided in the rqt context.
    """

    def __init__(self, node: Node):
        self._node = node

        self._get_exo_mode_array_client = self._node.create_client(GetExoModeArray, 'get_exo_mode_array')

        while not self._get_exo_mode_array_client.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().warn("Waiting for service get_exo_mode_array to become available...")

        self._node.get_logger().info("Connected to service get_exo_mode_array")
        

    def __del__(self):
        """Deconstructer, that shutsdown the publishers and resets the timers."""
        self._node.destroy_publisher(self._instruction_gait_pub)
        self._node.destroy_publisher(self._error_pub)
        if self._ping:
            self._alive_timer.shutdown()
            self._alive_timer.join()
            self._alive_pub.unregister()


    @property
    def node(self):
        """Define the node."""
        return self._node

    def publish_home_stand(self) -> None:
        """Use the service to instruct the exo to go to the home stand position."""
        request = GetExoModeArray.Request()
        request.desired_mode.mode = 1  # Set the mode to 1

        future = self._get_exo_mode_array_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)

        if future.result() is not None:
            self._node.get_logger().info('Mode sent: %d' % request.desired_mode.mode)
        else:
            self._node.get_logger().error('Exception while calling service: %r' % future.exception())
    

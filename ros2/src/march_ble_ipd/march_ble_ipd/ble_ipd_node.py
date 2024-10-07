import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from march_shared_msgs.srv import GetExoModeArray
from march_shared_msgs.msg import Alive

import getpass
import socket
import os 
import signal

from march_ble_ipd.bluetooth_server import BluetoothServer

COLOR = "\u001b[38;5;201m"
RESET = "\033[0m"
TIMEOUT = 30


class BLEInputDeviceNode(Node):

    ID_FORMAT = "ble@{machine}@{user}ros2"

    def __init__(self):
        super().__init__('bluetooth_input_device_node')
        self._requested_mode = GetExoModeArray.Request()
        self._available_modes_future = None
        self._connected = False 
        self._connected_first_time = False
        self._disconnect_timestamp = None
        self._disconnection_handled = False
        self._connection_handled = False
        self._current_mode = 3 # BOOTUP
        self._id = self.ID_FORMAT.format(machine=socket.gethostname(), user=getpass.getuser())

        self._get_exo_mode_array_client = self.create_client(GetExoModeArray, 'get_exo_mode_array')

        while not self._get_exo_mode_array_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for service get_exo_mode_array to become available...")

        self.get_logger().info("Connected to service get_exo_mode_array")

        self._alive_timer = self.create_timer(timer_period_sec=0.1,
                                                callback=self.check_connectivity,
                                                clock=self.get_clock())
        
        self.get_logger().info(f"{COLOR}Ready to connect to bluetooth device{RESET}")
        self._bluetooth_server = BluetoothServer(lambda mode: self.publish_mode(mode), self)  


    def publish_mode(self, mode: int) -> None:
        self._requested_mode.desired_mode.mode = mode
        self._available_modes_future = self._get_exo_mode_array_client.call_async(self._requested_mode)
        self.get_logger().info("Requested mode: " + str(mode))

        # Don't wait for the service call to complete
        self._available_modes_future.add_done_callback(self.store_available_modes)

    def store_available_modes(self, future) -> None:
        if future.result() is None:
            self.get_logger().warn("No available modes received")
            return
        
        available_modes = future.result().mode_array.modes
        mode_list = [exo_mode.mode for exo_mode in available_modes]
        self._current_mode = future.result().current_mode.mode

    def check_connectivity(self) -> None:
        """Check whether a bluetooth device is still connected"""
        if self._connected == True:
            if not self._connection_handled:  # Check if connection message has been printed
                self.get_logger().info("Device is connected")
                self._connection_handled = True  # Mark connection message as printed
                self._disconnection_handled = False  # Reset disconnection handled flag for future disconnections
        elif self._connected == False and self._connected_first_time and not self._disconnection_handled:
            self.get_logger().warn("Device disconnected")
            if self._current_mode not in (0, 1, 3, 12, 13, 14, 17):  # Exo is not in Sit, Stand, or BootUp, High Step 1, High Step 2, High Step 3, Train Sit
                self.get_logger().warn("Device disconnected during gait, sending Stand mode")
                self.publish_mode(1)                
            self._disconnection_handled = True  # Mark disconnection message as printed
            self._connection_handled = False  # Reset connection handled flag for future connections
        else:
            if self._disconnect_timestamp is not None:
                self.get_logger().info("Device is reconnected!")
            self._disconnect_timestamp = None
        


def main(args=None):
    rclpy.init(args=args)

    node = BLEInputDeviceNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
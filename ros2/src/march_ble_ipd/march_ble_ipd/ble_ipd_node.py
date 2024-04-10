import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from march_shared_msgs.srv import GetExoModeArray
from march_shared_msgs.msg import Alive

from march_ble_ipd.bluetooth_server import BluetoothServer


class BLEInputDeviceNode(Node):

    def __init__(self):
        super().__init__('bluetooth_input_device_node')
        self._requested_mode = GetExoModeArray.Request()
        self._available_modes_future = None

        self._get_exo_mode_array_client = self.create_client(GetExoModeArray, 'get_exo_mode_array')

        while not self._get_exo_mode_array_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Waiting for service get_exo_mode_array to become available...")

        self.get_logger().info("Connected to service get_exo_mode_array")

        self._alive_pub = self.create_publisher(Alive, "/march/input_device/alive", 10)
        self._alive_timer = self.create_timer(timer_period_sec=0.1,
                                                callback=self.alive_callback,
                                                clock=self.get_clock())
        
        
        self._bluetooth_server = BluetoothServer(lambda temperature: self.publish_mode(temperature))

    def publish_mode(self, mode: int) -> None:
        self._requested_mode.desired_mode.mode = mode
        self._available_modes_future = self._get_exo_mode_array_client.call_async(self._requested_mode)
        self.get_logger().info("Requested mode: " + str(mode))

    def store_available_modes(self, future) -> None:
        if future.result() is None:
            self._node.get_logger().warn("No available modes received")
            return
        
        available_modes = future.result().mode_array.modes
        mode_list = [exo_mode.mode for exo_mode in available_modes]
        print(mode_list)
    
    def alive_callback(self) -> None:
        """Callback to send out an alive message."""
        msg = Alive(stamp=self._node.get_clock().now().to_msg(), id=self._id)
        self._alive_pub.publish(msg)


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
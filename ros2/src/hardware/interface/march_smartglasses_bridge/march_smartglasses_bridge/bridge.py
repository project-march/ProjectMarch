"""Author: Thijs Raymakers, MVI."""
import rclpy
import socket
from rclpy.node import Node
from march_shared_msgs.msg import CurrentIPDState


class SmartglassBridge(Node):
    """Node to handle communication with the smart glasses."""

    def __init__(self):
        super().__init__("march_smartglass_bridge")
        self.get_logger().info("Creating smartglass bridge...")

        self._current_gait = self.create_subscription(
            msg_type=CurrentIPDState,
            topic="/march/input_device/current_state",
            callback=self.current_state_callback,
            qos_profile=1,
        )

        self.host = self.declare_parameter("hud_host").get_parameter_value().string_value
        self.port = self.declare_parameter("hud_port").get_parameter_value().integer_value

    def current_state_callback(self, msg: CurrentIPDState) -> None:
        """Callback function to update the smart glasses if the input_device current_state is updated."""
        self.get_logger().debug(f"Received IPD state message: {msg.menu_name}")
        self.send_to_smartglasses(msg.menu_name)

    def send_to_smartglasses(self, state: str) -> None:
        """Sends the new state to smart glasses."""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                host = socket.gethostbyname(self.host)
                s.connect((host, self.port))
                s.sendall(bytes(f"{state}\n", "utf-8"))
            except ConnectionRefusedError:
                self.get_logger().warn(f"Smart glasses can not be found on {self.host}:{self.port}")


def main(args=None):
    """Main script to start the smartglass node."""
    rclpy.init(args=args)
    node = SmartglassBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

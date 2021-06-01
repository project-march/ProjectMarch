import rclpy
import socket
from rclpy.node import Node
from march_shared_msgs.msg import CurrentIPDState

class SmartglassBridge(Node):
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
        self.get_logger().debug(f"Received IPD state message: {msg.menu_name}")
        self.send_to_smartglasses(msg.menu_name)

    def send_to_smartglasses(self, state: str):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                host = socket.gethostbyname(self.host)
                s.connect((host, self.port))
                s.sendall(bytes(f"{state}\n", "utf-8"))
            except ConnectionRefusedError:
                self.get_logger().warn(f"Smart glasses can not be found on {self.host}:{self.port}")

def main(args=None):
    rclpy.init(args=args)
    node = SmartglassBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

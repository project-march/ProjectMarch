"""Author: Jelmer de Wolde, MVII."""

import subprocess  # noqa
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class BeepNode(Node):
    """A node used to play beep sounds to inform the pilot about the gait."""

    def __init__(self):
        super().__init__("beep_npde")
        self.subscription = self.create_subscription(
            String, "/march/chosen_foot_position/feedback", self.beep_callback, 10
        )

    def beep_callback(self, msg: String):
        """beep."""
        if msg.data == "eeg_start":
            _beep(1, length=0.5)
        elif msg.data == "stop":
            _beep(3)
        elif msg.data == "long":
            _beep(2)
        elif msg.data == "short":
            _beep(1)


def main(args=None):
    """Main function starts the node."""
    rclpy.init(args=args)
    node = BeepNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


def _beep(beeps: int, length: float = 0.1):
    """Plays the given amount of beeps with the given length."""
    cmd = ["play", "-n", "synth", str(length), "sine", "880", "vol", "1.0"]
    for _n in range(beeps):
        subprocess.run(cmd, capture_output=True)  # noqa

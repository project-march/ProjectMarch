from rclpy.node import Node


class Logger:
    """Publishes a message on the logger of the node with the given prefix.
    Used to avoid confusion when logging on a node that is used by many files."""

    def __init__(self, node: Node, prefix: str):
        self.node = node
        self.msg_prefix = prefix

    def debug(self, msg: str) -> None:
        self.node.get_logger().debug(f"[{self.msg_prefix}] {msg}")

    def info(self, msg: str) -> None:
        self.node.get_logger().info(f"[{self.msg_prefix}] {msg}")

    def warning(self, msg: str) -> None:
        self.node.get_logger().warning(f"[{self.msg_prefix}] {msg}")

    def error(self, msg: str) -> None:
        self.node.get_logger().error(f"[{self.msg_prefix}] {msg}")

    def fatal(self, msg: str) -> None:
        self.node.get_logger().fatal(f"[{self.msg_prefix}] {msg}")

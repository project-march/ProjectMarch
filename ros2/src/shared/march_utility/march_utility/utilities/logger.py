"""Author: Marten Haitjema, MVII."""

from rclpy.node import Node


class Logger:
    """Publishes a message on the logger of the node with the given prefix.

    Used to avoid confusion when logging on a node that is used by many files.
    """

    def __init__(self, node: Node, prefix: str):
        self.node = node
        self.msg_prefix = prefix

    def debug(self, msg: str) -> None:
        """Logs the message with given prefix on DEBUG level."""
        self.node.get_logger().debug(f"[{self.msg_prefix}] {msg}")

    def info(self, msg: str) -> None:
        """Logs the message with given prefix on INFO level."""
        self.node.get_logger().info(f"[{self.msg_prefix}] {msg}")

    # support both warn and warning
    def warning(self, msg: str) -> None:
        """Logs the message with given prefix on WARNING level."""
        self.node.get_logger().warning(f"[{self.msg_prefix}] {msg}")

    def warn(self, msg: str) -> None:
        """Logs the message with given prefix on WARNING level."""
        self.node.get_logger().warning(f"[{self.msg_prefix}] {msg}")

    def error(self, msg: str) -> None:
        """Logs the message with given prefix on ERROR level."""
        self.node.get_logger().error(f"[{self.msg_prefix}] {msg}")

    def fatal(self, msg: str) -> None:
        """Logs the message with given prefix on FATAL level."""
        self.node.get_logger().fatal(f"[{self.msg_prefix}] {msg}")

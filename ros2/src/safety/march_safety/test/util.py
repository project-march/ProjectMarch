from threading import Event


class ErrorCounter:
    def __init__(self, node):
        self.count = 0
        self.node = node

    def cb(self, _):
        self.node.get_logger().info("Recieved message!")
        self.count += 1

from threading import Event


class ErrorCounter:
    def __init__(self, node, event: Event):
        self.count = 0
        self.node = node
        self.event = event

    def cb(self, _):
        self.node.get_logger().info("Recieved message!")
        self.count += 1
        self.event.set()

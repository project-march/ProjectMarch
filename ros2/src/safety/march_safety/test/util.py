class ErrorCounter:
    def __init__(self):
        self.count = 0

    def cb(self, _):
        self.count += 1

class GaitVersionToolError(Exception):
    def __init__(self, msg):
        super(GaitVersionToolError, self).__init__(msg)


class InvalidResponseError(GaitVersionToolError):
    def __init__(self):
        super(InvalidResponseError, self).__init__("Failed to parse response")


class GaitServiceError(GaitVersionToolError):
    def __init__(self):
        super(GaitServiceError, self).__init__("Failed to contact gait selection")

class GaitSelectionError(Exception):
    def __init__(self, msg):
        super(GaitSelectionError, self).__init__(msg)


class InvalidResponseError(GaitSelectionError):
    def __init__(self):
        super(InvalidResponseError, self).__init__('Failed to parse response')


class GaitServiceError(GaitSelectionError):
    def __init__(self):
        super(GaitServiceError, self).__init__('Failed to contact gait selection')

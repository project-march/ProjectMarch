from python_qt_binding.QtCore import QDateTime, QObject


class Message(QObject):

    DEBUG = 1
    INFO = 2
    WARN = 3
    ERROR = 4
    FATAL = 5

    LEVEL_LABELS = {
        DEBUG: 'Debug',
        INFO: 'Info',
        WARN: 'Warn',
        ERROR: 'Error',
        FATAL: 'Fatal',
    }

    def __init__(self, message, level=INFO):
        super(Message, self).__init__()

        self.message = message
        self.level = level

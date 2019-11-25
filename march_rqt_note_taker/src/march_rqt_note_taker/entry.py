from python_qt_binding.QtCore import QObject, QTime


class Entry(QObject):

    def __init__(self, content):
        super(Entry, self).__init__()

        self.content = content
        self.time = QTime.currentTime()

    def time_string(self):
        return self.time.toString()

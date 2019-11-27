from python_qt_binding.QtCore import QObject, QTime
from rosgraph_msgs.msg import Log


class Entry(QObject):

    def __init__(self, content, is_error=False):
        super(Entry, self).__init__()

        self.content = content
        self.time = QTime.currentTime()
        self.is_error = is_error

    @classmethod
    def from_ros_msg(cls, log_msg):
        """Returns an Entry from a given ROS log message.

        :type log_msg: rosgraph_msgs.msg.Log
        :param log_msg: The message to convert
        """
        return cls(log_msg.msg, log_msg.level >= Log.ERROR)

    def time_string(self):
        return self.time.toString()

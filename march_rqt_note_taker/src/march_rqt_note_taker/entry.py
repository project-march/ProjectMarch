from python_qt_binding.QtCore import QDateTime, QObject
from rosgraph_msgs.msg import Log


class Entry(QObject):

    def __init__(self, content, date_time=QDateTime.currentDateTime(), is_error=False):
        super(Entry, self).__init__()

        self.content = content
        self.date_time = date_time
        self.is_error = is_error

    @classmethod
    def from_ros_msg(cls, log_msg):
        """Returns an Entry from a given ROS log message.

        :type log_msg: rosgraph_msgs.msg.Log
        :param log_msg: The message to convert
        """
        date_time = (QDateTime.currentDateTime() if log_msg.header.stamp.is_zero() else
                     QDateTime.fromSecsSinceEpoch(log_msg.header.stamp.secs))
        return cls(log_msg.msg, date_time, log_msg.level >= Log.ERROR)

    def time_string(self):
        return self.date_time.toString('HH:mm:ss')

    def __str__(self):
        """Returns a string representation of an Entry."""
        return '[{0}] {1}'.format(self.time_string(), self.content)

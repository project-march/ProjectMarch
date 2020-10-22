from typing import Optional

from python_qt_binding.QtCore import QDateTime, QObject
from rcl_interfaces.msg import Log


class Entry(QObject):

    def __init__(self, content, date_time: Optional[QDateTime] = None, is_error: Optional[bool] = False):
        super(Entry, self).__init__()

        self.content = content
        self.date_time = QDateTime.currentDateTime() if date_time is None else date_time
        self.is_error = is_error

    @classmethod
    def from_ros_msg(cls, log_msg: Log):
        """Returns an Entry from a given ROS log message.

        :param log_msg: The message to convert
        """
        date_time = (QDateTime.currentDateTime() if log_msg.stamp.is_zero() else
                     QDateTime.fromSecsSinceEpoch(log_msg.stamp.secs))
        return cls(log_msg.msg, date_time, log_msg.level >= Log.ERROR)

    def time_string(self) -> str:
        return self.date_time.toString('HH:mm:ss')

    def __str__(self):
        """Returns a string representation of an Entry."""
        return '[{0}] {1}'.format(self.date_time.toString(), self.content)

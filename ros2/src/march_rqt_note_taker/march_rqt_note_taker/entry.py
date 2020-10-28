import sys
from typing import Optional

from python_qt_binding.QtCore import QDateTime, QObject

from rcl_interfaces.msg import Log


class Entry(QObject):

    def __init__(self, content: str, date_time: Optional[QDateTime] = None, is_error: Optional[bool] = False):
        """Construct an Entry.

        :param content Content of the message
        :param date_time Time the entry was created
        :param is_error Whether the entry corresponds to an error
        """
        super(Entry, self).__init__()

        self.content = content
        self.date_time = QDateTime.currentDateTime() if date_time is None else date_time
        self.is_error = is_error

    @classmethod
    def from_ros_msg(cls, log_msg: Log):
        """Returns an Entry from a given ROS log message.

        :param log_msg: The message to convert
        """
        # fromSecsSinceEpoch did not give the right time
        # if log_msg.stamp is None:
        #     date_time = QDateTime.currentDateTime()
        # else:
        #     date_time = QDateTime.fromSecsSinceEpoch(log_msg.stamp.sec)
        return cls(log_msg.msg, QDateTime.currentDateTime(), log_msg.level >=
                   int.from_bytes(Log.ERROR, sys.byteorder))

    def time_string(self) -> str:
        return self.date_time.toString('HH:mm:ss')

    def __str__(self):
        """Returns a string representation of an Entry."""
        return '[{0}] {1}'.format(self.date_time.toString(), self.content)

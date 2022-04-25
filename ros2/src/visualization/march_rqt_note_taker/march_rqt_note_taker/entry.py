"""Author: Olav de Haas, MIV."""
import sys
from typing import Optional

from python_qt_binding.QtCore import QDateTime, QObject

from rcl_interfaces.msg import Log


class Entry(QObject):
    """An Entry object with data for QtTableModel objects.

    Args:
        content (str): Content of the message.
        date_time (QDateTime, Optional): Time the entry was created. Defaults to None.
        is_error (bool, Optional): Whether the entry corresponds to an error. Defaults to False.
    """

    def __init__(self, content: str, date_time: Optional[QDateTime] = None, is_error: Optional[bool] = False):
        super(Entry, self).__init__()

        self.content = content
        self.date_time = QDateTime.currentDateTime() if date_time is None else date_time
        self.is_error = is_error

    @classmethod
    def from_ros_msg(cls, log_msg: Log, use_current_time: Optional[bool] = True):
        """Returns an Entry from a given ROS log message.

        Args:
            log_msg (Log): The message to convert.
            use_current_time (bool, Optional): Whether the current time should be used,
                instead of the timestamp of the log. Default is uses the current time.
        """
        if use_current_time or log_msg.stamp is None:
            date_time = QDateTime.currentDateTime()
        else:
            date_time = QDateTime.fromSecsSinceEpoch(log_msg.stamp.sec)
        return cls(
            log_msg.msg,
            date_time,
            log_msg.level >= int.from_bytes(Log.ERROR, sys.byteorder),
        )

    def time_string(self) -> str:
        """Returns the time of the enry in the format: `HH:mm:ss`."""
        return self.date_time.toString("HH:mm:ss")

    def __str__(self):
        """Returns a string representation of an Entry."""
        return "[{0}] {1}".format(self.date_time.toString(), self.content)

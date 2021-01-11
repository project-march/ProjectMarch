import sys
import unittest

from python_qt_binding.QtCore import QDateTime, QTimeZone
from rcl_interfaces.msg import Log
from builtin_interfaces.msg import Time

from march_rqt_note_taker.entry import Entry


class EntryTest(unittest.TestCase):
    def test_create_no_error(self):
        entry = Entry("")
        self.assertFalse(entry.is_error)

    def test_create_with_content(self):
        content = "test"
        entry = Entry(content)
        self.assertEqual(entry.content, content)

    def test_create_from_info_log_msg(self):
        content = "test"
        msg = Log()
        msg.msg = content
        msg.level = int.from_bytes(Log.INFO, sys.byteorder)
        entry = Entry.from_ros_msg(msg)
        self.assertEqual(entry.content, content)
        self.assertFalse(entry.is_error)
        self.assertAlmostEqual(
            entry.date_time.toSecsSinceEpoch(), QDateTime.currentSecsSinceEpoch(), 10
        )

    def test_create_from_error_log_msg(self):
        msg = Log()
        msg.level = int.from_bytes(Log.ERROR, sys.byteorder)
        entry = Entry.from_ros_msg(msg)
        self.assertTrue(entry.is_error)

    def test_create_from_fatal_log_msg(self):
        msg = Log()
        msg.level = int.from_bytes(Log.FATAL, sys.byteorder)
        entry = Entry.from_ros_msg(msg)
        self.assertTrue(entry.is_error)

    def test_create_from_log_msg_with_stamp(self):
        seconds_since_epoch = 100
        msg = Log()
        msg.stamp = Time(sec=seconds_since_epoch, nanosec=0)
        msg.level = int.from_bytes(Log.DEBUG, sys.byteorder)
        entry = Entry.from_ros_msg(msg, use_current_time=False)
        self.assertEqual(entry.date_time.toSecsSinceEpoch(), seconds_since_epoch)

    def test_to_string(self):
        date_time = QDateTime.fromSecsSinceEpoch(2)
        content = "test"
        entry = Entry(content, date_time)
        self.assertEqual(str(entry), "[{0}] {1}".format(date_time.toString(), content))

    def test_to_time_string(self):
        date_time = QDateTime.fromSecsSinceEpoch(5, QTimeZone.utc())
        entry = Entry("", date_time)
        self.assertEqual(entry.time_string(), "00:00:05")

import unittest

from python_qt_binding.QtCore import QDateTime
from rosgraph_msgs.msg import Log
import rospy

from march_rqt_note_taker.entry import Entry


class EntryTest(unittest.TestCase):
    def test_create_no_error(self):
        entry = Entry('')
        self.assertFalse(entry.is_error)

    def test_create_with_content(self):
        content = 'test'
        entry = Entry(content)
        self.assertEqual(entry.content, content)

    def test_create_from_info_log_msg(self):
        content = 'test'
        msg = Log()
        msg.msg = content
        msg.level = Log.INFO
        entry = Entry.from_ros_msg(msg)
        self.assertEqual(entry.content, content)
        self.assertFalse(entry.is_error)
        self.assertAlmostEqual(entry.date_time.toSecsSinceEpoch(),
                               QDateTime.currentSecsSinceEpoch(),
                               10)

    def test_create_from_error_log_msg(self):
        msg = Log()
        msg.level = Log.ERROR
        entry = Entry.from_ros_msg(msg)
        self.assertTrue(entry.is_error)

    def test_create_from_fatal_log_msg(self):
        msg = Log()
        msg.level = Log.FATAL
        entry = Entry.from_ros_msg(msg)
        self.assertTrue(entry.is_error)

    def test_create_from_log_msg_with_stamp(self):
        seconds_since_epoch = 100
        msg = Log()
        msg.header.stamp = rospy.Time(secs=seconds_since_epoch, nsecs=0)
        msg.level = Log.DEBUG
        entry = Entry.from_ros_msg(msg)
        self.assertEqual(entry.date_time.toSecsSinceEpoch(),
                         seconds_since_epoch)

    def test_to_string(self):
        date_time = QDateTime.fromSecsSinceEpoch(2)
        content = 'test'
        entry = Entry(content, date_time)
        self.assertEqual(str(entry),
                         '[{0}] {1}'.format(date_time.toString(), content))

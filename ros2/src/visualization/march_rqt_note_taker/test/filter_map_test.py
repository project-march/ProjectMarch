import sys
import unittest

from rcl_interfaces.msg import Log

from march_rqt_note_taker.filter_map import FilterMap


class FilterMapTest(unittest.TestCase):
    def setUp(self):
        self.filter_map = FilterMap()
        self.log_msg = Log()

    def test_no_filters(self):
        self.assertIsNone(self.filter_map(self.log_msg))

    def test_accept_one_filter(self):
        self.filter_map.add_filter(lambda m: m == self.log_msg)
        self.assertIsNotNone(self.filter_map(self.log_msg))

    def test_reject_one_filter(self):
        self.filter_map.add_filter(lambda _: False)
        self.assertIsNone(self.filter_map(self.log_msg))

    def test_reject_one_filter_with_map(self):
        msg = "test"
        self.log_msg.msg = msg
        self.filter_map.add_filter(lambda m: False, lambda _: "")
        self.assertIsNone(self.filter_map(self.log_msg))
        self.assertEqual(self.log_msg.msg, msg)

    def test_accept_at_least_one_filter(self):
        self.filter_map.add_filter(lambda _: True)
        self.filter_map.add_filter(lambda _: False)
        self.assertIsNotNone(self.filter_map(self.log_msg))

    def test_reject_multiple_filters(self):
        self.filter_map.add_filter(lambda _: False)
        self.filter_map.add_filter(lambda _: False)
        self.assertIsNone(self.filter_map(self.log_msg))

    def test_accept_log_level_filter(self):
        level = Log.DEBUG
        self.log_msg.level = int.from_bytes(Log.DEBUG, sys.byteorder)
        self.filter_map.add_filter_on_level(level)
        self.assertIsNotNone(self.filter_map(self.log_msg))

    def test_reject_log_level_filter(self):
        level = Log.INFO
        self.log_msg.level = int.from_bytes(Log.DEBUG, sys.byteorder)
        self.filter_map.add_filter_on_level(level)
        self.assertIsNone(self.filter_map(self.log_msg))

    def test_accept_and_map(self):
        mapped_msg = "test"
        self.filter_map.add_filter(lambda _: True, lambda _: mapped_msg)
        self.log_msg = self.filter_map(self.log_msg)
        self.assertEqual(self.log_msg.msg, mapped_msg)

    def test_accept_and_map_on_log_level(self):
        mapped_msg = "test"
        level = Log.DEBUG
        self.log_msg.level = int.from_bytes(level, sys.byteorder)
        self.filter_map.add_filter_on_level(level, msg_map=lambda _: mapped_msg)
        self.log_msg = self.filter_map(self.log_msg)
        self.assertEqual(self.log_msg.msg, mapped_msg)

import unittest

from rosgraph_msgs.msg import Log

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
        msg = 'test'
        self.log_msg.msg = msg
        self.filter_map.add_filter(lambda m: False, lambda _: '')
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
        self.log_msg.level = level
        self.filter_map.add_filter_on_level(lambda _: True, level)
        self.assertIsNotNone(self.filter_map(self.log_msg))

    def test_reject_log_level_filter(self):
        level = Log.INFO
        self.log_msg.level = Log.DEBUG
        self.filter_map.add_filter_on_level(lambda _: True, level)
        self.assertIsNone(self.filter_map(self.log_msg))

    def test_info_level_filter(self):
        self.log_msg.level = Log.INFO
        self.filter_map.add_filter_info_level(lambda _: True)
        self.assertIsNotNone(self.filter_map(self.log_msg))

    def test_accept_and_map(self):
        mapped_msg = 'test'
        self.filter_map.add_filter(lambda _: True, lambda _: mapped_msg)
        self.log_msg = self.filter_map(self.log_msg)
        self.assertEqual(self.log_msg.msg, mapped_msg)

    def test_accept_and_map_on_log_level(self):
        mapped_msg = 'test'
        level = Log.DEBUG
        self.log_msg.level = level
        self.filter_map.add_filter_on_level(lambda _: True, level, lambda _: mapped_msg)
        self.log_msg = self.filter_map(self.log_msg)
        self.assertEqual(self.log_msg.msg, mapped_msg)

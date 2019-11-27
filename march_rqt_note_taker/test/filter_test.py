import unittest

from march_rqt_note_taker.filter import Filter
from rosgraph_msgs.msg import Log


class FilterTest(unittest.TestCase):
    def setUp(self):
        self.filter = Filter()
        self.msg = Log()

    def test_no_filters(self):
        self.assertFalse(self.filter.apply(self.msg))

    def test_accept_one_filter(self):
        self.filter.add_filter(lambda m: m == self.msg)
        self.assertTrue(self.filter.apply(self.msg))

    def test_reject_one_filter(self):
        self.filter.add_filter(lambda _: False)
        self.assertFalse(self.filter.apply(self.msg))

    def test_accept_at_least_one_filter(self):
        self.filter.add_filter(lambda _: True)
        self.filter.add_filter(lambda _: False)
        self.assertTrue(self.filter.apply(self.msg))

    def test_reject_multiple_filters(self):
        self.filter.add_filter(lambda _: False)
        self.filter.add_filter(lambda _: False)
        self.assertFalse(self.filter.apply(self.msg))

    def test_accept_log_level_filter(self):
        level = Log.DEBUG
        self.msg.level = level
        self.filter.add_filter_on_level(lambda _: True, level)
        self.assertTrue(self.filter.apply(self.msg))

    def test_reject_log_level_filter(self):
        level = Log.INFO
        self.msg.level = Log.DEBUG
        self.filter.add_filter_on_level(lambda _: True, level)
        self.assertFalse(self.filter.apply(self.msg))

import unittest

from rosgraph_msgs.msg import Log

from march_rqt_note_taker.entry import Entry
from march_rqt_note_taker.entry_model import EntryModel


class EntryModelTest(unittest.TestCase):
    def setUp(self):
        self.model = EntryModel()

    def test_empty_model(self):
        self.assertEqual(self.model.rowCount(), 0)
        self.assertEqual(self.model.columnCount(), 2)

    def test_insert_single_row(self):
        self.model.insert_row(Entry('test'))
        self.assertEqual(self.model.rowCount(), 1)

    def test_insert_multiple_row(self):
        self.model.insert_row(Entry('test1'))
        self.model.insert_row(Entry('test2'))
        self.model.insert_row(Entry('test3'))
        self.assertEqual(self.model.rowCount(), 3)

    def test_insert_log_msg(self):
        msg = 'test'
        log_msg = Log(msg=msg)
        self.model.insert_log_msg(log_msg)
        self.assertEqual(self.model.rowCount(), 1)

        index = self.model.createIndex(0, 1)
        self.assertEqual(self.model.data(index), msg)

    def test_remove_one_row(self):
        self.model.insert_row(Entry('test'))
        self.model.remove_rows(0)
        self.assertEqual(self.model.rowCount(), 0)

    def test_remove_multiple_rows(self):
        self.model.insert_row(Entry('test1'))
        self.model.insert_row(Entry('test2'))
        self.model.insert_row(Entry('test3'))
        self.model.remove_rows(1, 2)
        self.assertEqual(self.model.rowCount(), 1)
        self.assertEqual(self.model.data(self.model.createIndex(0, 1)), 'test1')

    def test_remove_rows_out_of_range(self):
        self.model.insert_row(Entry('test2'))
        self.model.remove_rows(2, 2)
        self.assertEqual(self.model.rowCount(), 1)

    def test_remove_too_many_rows(self):
        self.model.insert_row(Entry('test'))
        self.model.remove_rows(0, 2)
        self.assertEqual(self.model.rowCount(), 0)

    def test_get_row_out_of_bounds(self):
        self.assertIsNone(self.model.data(self.model.createIndex(0, 1)))

    def test_get_column_out_of_bounds(self):
        self.assertIsNone(self.model.data(self.model.createIndex(0, len(self.model.columns))))

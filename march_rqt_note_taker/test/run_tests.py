#!/usr/bin/env python
import rosunit

from .entry_model_test import EntryModelTest
from .filter_map_test import FilterMapTest

PKG = 'march_rqt_note_taker'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_filter', FilterMapTest)
    rosunit.unitrun(PKG, 'test_entry_model', EntryModelTest)

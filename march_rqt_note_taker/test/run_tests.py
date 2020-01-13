#!/usr/bin/env python
import rosunit

from .entry_model_test import EntryModelTest
from .entry_test import EntryTest
from .filter_map_test import FilterMapTest

PKG = 'march_rqt_note_taker'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'entry_mode_test', EntryModelTest)
    rosunit.unitrun(PKG, 'entry_test', EntryTest)
    rosunit.unitrun(PKG, 'filter_test', FilterMapTest)

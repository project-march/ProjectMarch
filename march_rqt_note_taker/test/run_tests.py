#!/usr/bin/env python
import rosunit

from .filter_test import FilterTest

PKG = 'march_rqt_note_taker'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'test_filter', FilterTest)

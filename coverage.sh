#!/usr/bin/env sh

cd build/march_rqt_note_taker || return
coverage xml --rcfile ../../src/monitor/.coveragerc
cd ../.. || return

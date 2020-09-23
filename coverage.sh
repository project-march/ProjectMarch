#!/usr/bin/env sh

<<<<<<< HEAD
cd build/march_rqt_note_taker || return
coverage xml --rcfile ../../src/monitor/.coveragerc
=======
cd build/march_rqt_gait_generator || return
coverage xml --rcfile ../../src/gait-generation/.coveragerc
>>>>>>> gait-generation/develop
cd ../.. || return

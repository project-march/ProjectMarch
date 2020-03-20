#!/usr/bin/env python
from nosetests.modifiable_joint_trajectory_test import ModifiableJointTrajectoryTest
from nosetests.modifiable_setpoint_test import ModifiableSetpointTest
from nosetests.modifiable_subgait_test import ModifiableSubgaitTest
from nosetests.gait_generator_controller_test import GaitGeneratorControllerTest
import rosunit

PKG = 'march_rqt_gait_generator'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'modifiable_setpoint_test', ModifiableSetpointTest)
    rosunit.unitrun(PKG, 'modifiable_joint_trajectory_test', ModifiableJointTrajectoryTest)
    rosunit.unitrun(PKG, 'modifiable_subgait_test', ModifiableSubgaitTest)
    rosunit.unitrun(PKG, 'gait_generator_controller_test', GaitGeneratorControllerTest)

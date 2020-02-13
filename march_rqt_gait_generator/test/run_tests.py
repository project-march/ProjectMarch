#!/usr/bin/env python
import rosunit

from .modifiable_joint_trajectory_test import ModifiableJointTrajectoryTest
from .modifiable_setpoint_test import ModifiableSetpointTest
from .modifiable_subgait_test import ModifiableSubgaitTest

PKG = 'march_shared_classes'

if __name__ == '__main__':
    rosunit.unitrun(PKG, 'setpoint_test', ModifiableSetpointTest)
    rosunit.unitrun(PKG, 'joint_trajectory_test', ModifiableJointTrajectoryTest)
    rosunit.unitrun(PKG, 'subgait_test', ModifiableSubgaitTest)

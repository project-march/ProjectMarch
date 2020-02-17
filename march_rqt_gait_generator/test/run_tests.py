#!/usr/bin/env python
import rostest
import rospy
from nosetests.modifiable_joint_trajectory_test import ModifiableJointTrajectoryTest
from nosetests.modifiable_setpoint_test import ModifiableSetpointTest
from nosetests.modifiable_subgait_test import ModifiableSubgaitTest

PKG = 'march_rqt_gait_generator'

if __name__ == '__main__':
    rostest.rosrun(PKG, 'modifiable_setpoint_test', ModifiableSetpointTest)
    rostest.rosrun(PKG, 'modifiable_joint_trajectory_test', ModifiableJointTrajectoryTest)
    rostest.rosrun(PKG, 'modifiable_subgait_test', ModifiableSubgaitTest)

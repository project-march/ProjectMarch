import unittest

import numpy as np

from march_rqt_gait_generator.model.modifiable_joint_trajectory import ModifiableJointTrajectory
from march_shared_classes.gait.limits import Limits
from march_rqt_gait_generator.model.modifiable_setpoint import ModifiableSetpoint


class ModifiableJointTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.joint_name = 'test_joint'
        self.limits = Limits(-1, 1, 2)
        self.duration = 2.0
        self.times = [0, self.duration / 2.0, self.duration]
        self.setpoints = [ModifiableSetpoint(t, 2 * t, t / 2.0) for t in self.times]
        self.joint_trajectory = ModifiableJointTrajectory(self.joint_name, self.limits, self.setpoints, self.duration)

    def test_something(self):
        self.assertTrue(True)

import unittest

from march_rqt_gait_generator.model.modifiable_setpoint import ModifiableSetpoint


class ModifiableSetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = ModifiableSetpoint(1.123412541, 0.034341255, 123.162084549)

    def test_invert_time(self):
        self.setpoint.invert(2)
        self.assertAlmostEqual(self.setpoint.time, 2 - 1.123412541, 8)

    def test_invert_position(self):
        self.setpoint.invert(2)
        self.assertAlmostEqual(self.setpoint.position, 0.034341255, 8)

    def test_invert_velocity(self):
        self.setpoint.invert(2)
        self.assertAlmostEqual(self.setpoint.velocity, -123.162084549, 8)

import unittest

from march_rqt_gait_generator.model.modifiable_setpoint import ModifiableSetpoint


class ModifiableSetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = ModifiableSetpoint(1.123412541, 0.0343412512, 123.162084)

    def test_invert_time(self):
        self.setpoint.invert(2)
        self.assertEqual(self.setpoint.time, 2 - 1.1234)

    def test_invert_position(self):
        self.setpoint.invert(2)
        self.assertEqual(self.setpoint.position, 0.0343)

    def test_invert_velocity(self):
        self.setpoint.invert(2)
        self.assertEqual(self.setpoint.velocity, -123.1621)

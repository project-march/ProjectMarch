import unittest

from march_rqt_gait_generator.model.modifiable_setpoint import ModifiableSetpoint


class ModifiableSetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = ModifiableSetpoint(1.123412541, 0.0343412512, 123.162084)

    def test_something(self):
        self.assertTrue(True)

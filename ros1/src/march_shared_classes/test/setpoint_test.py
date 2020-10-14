import unittest

from march_shared_classes.gait.setpoint import Setpoint


class SetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = Setpoint(1.123412541, 0.0343412512, 123.162084)
        self.setpoint_dict = {'left_hip_aa': self.setpoint,
                               'left_hip_fe': self.setpoint,
                               'left_knee': self.setpoint,
                               'right_hip_aa': self.setpoint,
                               'right_hip_fe': self.setpoint,
                               'right_knee': self.setpoint,
                               'left_ankle': self.setpoint,
                               'right_ankle': self.setpoint}

    def test_time_rounding(self):
        self.assertEqual(self.setpoint.time, 1.1234)

    def test_position_rounding(self):
        self.assertEqual(self.setpoint.position, 0.0343)

    def test_velocity_rounding(self):
        self.assertEqual(self.setpoint.velocity, 123.1621)

    def test_string_output(self):
        self.assertEqual(str(self.setpoint), 'Time: %s, Position: %s, Velocity: %s' % (1.1234, 0.0343, 123.1621))

    def test_equal(self):
        other_setpoint = Setpoint(1.123410541132, 0.03433998, 123.1621)
        self.assertEqual(self.setpoint, other_setpoint)

    def test_different_classes(self):
        other_setpoint = object()
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_unequal_time(self):
        other_setpoint = Setpoint(1.12349, 0.0343, 123.1621)
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_unequal_position(self):
        other_setpoint = Setpoint(1.1234, -0.0343, 123.1621)
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_unequal_velocity(self):
        other_setpoint = Setpoint(1.1234, 0.0343, 0)
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_interpolation_correct(self):
        parameter = 0.3
        other_setpoint = Setpoint(1, 1, 1)
        expected_result = Setpoint(self.setpoint.time * parameter + (1 - parameter) * 1,
                                   self.setpoint.position * parameter + (1 - parameter) * 1,
                                   self.setpoint.velocity * parameter + (1 - parameter) * 1)
        self.assertEqual(expected_result, Setpoint.interpolate_setpoints(self.setpoint, other_setpoint, parameter))


    def test_inverse_kinematics_position(self):
        foot_pos = Setpoint.get_foot_pos_from_angles(self.setpoint_dict)
        angles = Setpoint.get_angles_from_pos(foot_pos[0], 'left')\
                 + Setpoint.get_angles_from_pos(foot_pos[1], 'right')
        self.assertEqual([round(angle, 4) for angle in angles], [self.setpoint.position]*6)

    def test_inverse_kinematics_velocity(self):
        foot_vel = Setpoint.get_foot_pos_from_angles(self.setpoint_dict, velocity=True)
        angles = Setpoint.get_angles_from_pos(foot_vel[0], 'left') \
                 + Setpoint.get_angles_from_pos(foot_vel[1], 'right')
        self.assertEqual([round(angle, 4) for angle in angles], [self.setpoint.velocity]*6)






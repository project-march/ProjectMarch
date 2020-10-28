import unittest
import numpy as np

from march_shared_classes.gait.setpoint import Setpoint

VELOCITY_SCALE = 250


class SetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = Setpoint(1.123412541, 0.0343412512, 123.162084)  # 0.0343412512 123.162084
        self.setpoint_dict = {'left_hip_aa': self.setpoint,
                              'left_hip_fe': self.setpoint,
                              'left_knee': self.setpoint,
                              'right_hip_aa': self.setpoint,
                              'right_hip_fe': self.setpoint,
                              'right_knee': self.setpoint}

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
        angles = Setpoint.get_angles_from_pos(foot_pos[0], 'left') + \
            Setpoint.get_angles_from_pos(foot_pos[1], 'right')
        self.assertEqual([round(angle, 4) for angle in angles], [self.setpoint.position] * 6)

    def test_inverse_kinematics_reversed_position(self):
        desired_pos = [[0.0, -0.08, 0.6], [0.0, 0.08, 0.6]]
        new_angles = Setpoint.get_angles_from_pos(desired_pos[0], 'left') + \
            Setpoint.get_angles_from_pos(desired_pos[1], 'right')
        time = 1.0
        new_vel = 2.0
        resulting_angles = {'left_hip_aa': Setpoint(time, new_angles[0], new_vel),
                            'left_hip_fe': Setpoint(time, new_angles[1], new_vel),
                            'left_knee': Setpoint(time, new_angles[2], new_vel),
                            'right_hip_aa': Setpoint(time, new_angles[3], new_vel),
                            'right_hip_fe': Setpoint(time, new_angles[4], new_vel),
                            'right_knee': Setpoint(time, new_angles[5], new_vel)}
        resulting_pos = Setpoint.get_foot_pos_from_angles(resulting_angles)
        for i in range(0, len(resulting_pos)):
            for j in range(0, len(resulting_pos[i])):
                resulting_pos[i][j] = round(resulting_pos[i][j], 2)
        self.assertEqual(resulting_pos, desired_pos)

    def test_inverse_kinematics_velocity(self):
        foot_vel = np.array(Setpoint.get_foot_pos_from_angles(self.setpoint_dict, velocity=True))
        foot_pos = np.array(Setpoint.get_foot_pos_from_angles(self.setpoint_dict, velocity=False))

        new_angles_pos = [Setpoint.get_angles_from_pos(foot_pos[0], 'left'),
                          Setpoint.get_angles_from_pos(foot_pos[1], 'right')]

        new_angles_vel = (- np.array(new_angles_pos) +
                          + np.array([Setpoint.get_angles_from_pos(foot_pos[0] + foot_vel[0] / VELOCITY_SCALE, 'left'),
                                      Setpoint.get_angles_from_pos(foot_pos[1] + foot_vel[1] / VELOCITY_SCALE, 'right')
                                      ])) * VELOCITY_SCALE

        for i in range(0, 2):
            for j in range(0, 3):
                new_angles_vel[i, j] = round(new_angles_vel[i, j], 4)
        self.assertEqual(new_angles_vel.tolist(), [[self.setpoint.velocity] * 3, [self.setpoint.velocity] * 3])

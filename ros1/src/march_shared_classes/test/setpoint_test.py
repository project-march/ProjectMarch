import unittest

from march_shared_classes.gait.setpoint import Setpoint


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
        expected_result = Setpoint(self.setpoint.time * (1 - parameter) + parameter * 1,
                                   self.setpoint.position * (1 - parameter) + parameter * 1,
                                   self.setpoint.velocity * (1 - parameter) + parameter * 1)
        self.assertEqual(expected_result, Setpoint.interpolate_setpoints(self.setpoint, other_setpoint, parameter))

    def test_inverse_kinematics_position_left(self):
        foot_pos = Setpoint.get_foot_pos_from_angles(self.setpoint_dict)
        angles_left = Setpoint.calculate_joint_angles_from_foot_position(foot_pos, 'left')
        for key in angles_left.keys():
            self.assertAlmostEqual(angles_left[key], self.setpoint.position, places=4)

    def test_inverse_kinematics_position_right(self):
        foot_pos = Setpoint.get_foot_pos_from_angles(self.setpoint_dict)
        angles_right = Setpoint.calculate_joint_angles_from_foot_position(foot_pos, 'right')
        for key in angles_right.keys():
            self.assertAlmostEqual(angles_right[key], self.setpoint.position, places=4)

    def test_inverse_kinematics_reversed_position(self):
        desired_position = {'left_foot_x': 0.18, 'left_foot_y': -0.08, 'left_foot_z': 0.6,
                            'right_foot_x': 0.18, 'right_foot_y': 0.08, 'right_foot_z': 0.6}
        new_angles_left = Setpoint.calculate_joint_angles_from_foot_position(desired_position, 'left')
        new_angles_right = Setpoint.calculate_joint_angles_from_foot_position(desired_position, 'right')
        time = 1.0
        new_vel = 2.0
        resulting_angles = {'left_hip_aa': Setpoint(time, new_angles_left['left_hip_aa'], new_vel),
                            'left_hip_fe': Setpoint(time, new_angles_left['left_hip_fe'], new_vel),
                            'left_knee': Setpoint(time, new_angles_left['left_knee'], new_vel),
                            'right_hip_aa': Setpoint(time, new_angles_right['right_hip_aa'], new_vel),
                            'right_hip_fe': Setpoint(time, new_angles_right['right_hip_fe'], new_vel),
                            'right_knee': Setpoint(time, new_angles_right['right_knee'], new_vel)}
        resulting_position = Setpoint.get_foot_pos_from_angles(resulting_angles)
        for key in desired_position.keys():
            print(key)
            self.assertAlmostEqual(desired_position[key], resulting_position[key], places=4)

    def test_inverse_kinematics_velocity(self):
        foot_vel = Setpoint.get_foot_pos_from_angles(self.setpoint_dict, velocity=True)
        foot_pos = Setpoint.get_foot_pos_from_angles(self.setpoint_dict, velocity=False)

        new_angles_vel_left = Setpoint.calculate_joint_angles_from_foot_position(foot_pos, 'left', foot_vel)
        new_angles_vel_right = Setpoint.calculate_joint_angles_from_foot_position(foot_pos, 'right', foot_vel)

        for key in new_angles_vel_right.keys():
            if key.endswith('_velocity'):
                self.assertAlmostEqual(new_angles_vel_right[key], self.setpoint.velocity, places=4)
        for key in new_angles_vel_left.keys():
            if key.endswith('_velocity'):
                self.assertAlmostEqual(new_angles_vel_left[key], self.setpoint.velocity, places=4)

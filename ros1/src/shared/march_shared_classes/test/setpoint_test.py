import unittest

from march_shared_classes.foot_classes.feet_state import FeetState
from march_shared_classes.foot_classes.foot import Foot
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.utilities.side import Side
from march_shared_classes.utilities.vector_3d import Vector3d


class SetpointTest(unittest.TestCase):
    def setUp(self):
        self.setpoint = Setpoint(1.123412541, 0.034341255, 123.162084549)  # 0.0343412512 123.16208454
        self.setpoint_dict = {'left_hip_aa': self.setpoint,
                              'left_hip_fe': self.setpoint,
                              'left_knee': self.setpoint,
                              'right_hip_aa': self.setpoint,
                              'right_hip_fe': self.setpoint,
                              'right_knee': self.setpoint}

    def test_time_rounding(self):
        self.assertEqual(self.setpoint.time, 1.12341254)

    def test_position_rounding(self):
        self.assertEqual(self.setpoint.position, 0.03434126)

    def test_velocity_rounding(self):
        self.assertEqual(self.setpoint.velocity, 123.16208455)

    def test_string_output(self):
        self.assertEqual(str(self.setpoint), 'Time: %s, Position: %s, Velocity: %s' % (1.12341254, 0.03434126,
                                                                                       123.16208455))

    def test_equal(self):
        other_setpoint = Setpoint(1.1234125445313287, 0.034341264534, 123.16208455453)
        self.assertEqual(self.setpoint, other_setpoint)

    def test_different_classes(self):
        other_setpoint = object()
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_unequal_time(self):
        other_setpoint = Setpoint(1.123491381, 0.03434126, 123.16208455)
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_unequal_position(self):
        other_setpoint = Setpoint(1.12341254, -0.03434126, 123.16208455)
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_unequal_velocity(self):
        other_setpoint = Setpoint(1.12341254, 0.03434126, 0)
        self.assertNotEqual(self.setpoint, other_setpoint)

    def test_interpolation_correct(self):
        parameter = 0.3
        other_setpoint = Setpoint(1, 1, 1)
        expected_result = Setpoint(self.setpoint.time * (1 - parameter) + parameter * 1,
                                   self.setpoint.position * (1 - parameter) + parameter * 1,
                                   self.setpoint.velocity * (1 - parameter) + parameter * 1)
        self.assertEqual(expected_result, Setpoint.interpolate_setpoints(self.setpoint, other_setpoint, parameter))

    def test_inverse_kinematics_position(self):
        feet_state = FeetState.from_setpoints(self.setpoint_dict)
        new_setpoints = FeetState.feet_state_to_setpoint(feet_state)
        for key in new_setpoints.keys():
            self.assertAlmostEqual(new_setpoints[key].position, self.setpoint_dict[key].position, places=4)

    def test_inverse_kinematics_velocity(self):
        feet_state = FeetState.from_setpoints(self.setpoint_dict)
        new_setpoints = FeetState.feet_state_to_setpoint(feet_state)
        for key in new_setpoints.keys():
            self.assertAlmostEqual(new_setpoints[key].velocity, self.setpoint_dict[key].velocity, places=4)

    def test_inverse_kinematics_time(self):
        feet_state = FeetState.from_setpoints(self.setpoint_dict)
        new_setpoints = FeetState.feet_state_to_setpoint(feet_state)
        for key in new_setpoints.keys():
            self.assertAlmostEqual(new_setpoints[key].time, self.setpoint_dict[key].time, places=4)

    def test_inverse_kinematics_reversed_position(self):
        right_foot = Foot(Side.right, Vector3d(0.18, 0.08, 0.6), Vector3d(0, 0, 0))
        left_foot = Foot(Side.left, Vector3d(0.18, -0.08, 0.6), Vector3d(0, 0, 0))
        desired_state = FeetState(right_foot, left_foot, 0.1)
        new_setpoints = FeetState.feet_state_to_setpoint(desired_state)
        resulting_position = FeetState.from_setpoints(new_setpoints)

        dif_left = desired_state.left_foot.position - resulting_position.left_foot.position
        dif_right = desired_state.right_foot.position - resulting_position.right_foot.position
        self.assertTrue((dif_left.norm() < 0.00001))
        self.assertTrue((dif_right.norm() < 1 / 0.00001))

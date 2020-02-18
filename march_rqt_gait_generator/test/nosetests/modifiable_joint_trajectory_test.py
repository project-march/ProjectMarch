import unittest

from mock import Mock

from march_rqt_gait_generator.model.modifiable_joint_trajectory import ModifiableJointTrajectory
from march_rqt_gait_generator.model.modifiable_setpoint import ModifiableSetpoint
from march_shared_classes.gait.limits import Limits


class ModifiableJointTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.gait_generator = Mock()
        self.joint_name = 'test_joint'
        self.limits = Limits(-1, 1, 2)
        self.duration = 2.0
        self.times = [0, self.duration / 2.0, self.duration]
        self.setpoints = [ModifiableSetpoint(t, 0.5 * t, t) for t in self.times]
        self.joint_trajectory = ModifiableJointTrajectory(self.joint_name, self.limits, self.setpoints,
                                                          self.duration, self.gait_generator)

    # enforce_limits tests
    def test_enforce_limits_nonzero_start(self):
        self.joint_trajectory.setpoints[0].time = 0.1
        self.joint_trajectory.enforce_limits()
        self.assertEqual(self.joint_trajectory.setpoints[0].time, 0)

    def test_enforce_limits_nonduration_end(self):
        self.joint_trajectory.setpoints[-1].time = self.duration - 0.1
        self.joint_trajectory.enforce_limits()
        self.assertEqual(self.joint_trajectory.setpoints[-1].time, self.duration)

    def test_enforce_limits_max_position(self):
        self.joint_trajectory.setpoints[-1].position = self.limits.upper + 1
        self.joint_trajectory.enforce_limits()
        self.assertEqual(self.joint_trajectory.setpoints[-1].position, self.limits.upper)

    def test_enforce_limits_min_position(self):
        self.joint_trajectory.setpoints[1].position = self.limits.lower - 1
        self.joint_trajectory.enforce_limits()
        self.assertEqual(self.joint_trajectory.setpoints[1].position, self.limits.lower)

    def test_enforce_limits_max_velocity(self):
        self.joint_trajectory.setpoints[0].velocity = self.limits.velocity + 1
        self.joint_trajectory.enforce_limits()
        self.assertEqual(self.joint_trajectory.setpoints[0].velocity, self.limits.velocity)

    def test_enforce_limits_min_velocity(self):
        self.joint_trajectory.setpoints[-1].velocity = -self.limits.velocity - 1
        self.joint_trajectory.enforce_limits()
        self.assertEqual(self.joint_trajectory.setpoints[-1].velocity, -self.limits.velocity)

    # set_setpoints test
    def test_set_setpoints(self):
        new_setpoints = [ModifiableSetpoint(0.5 * t, 0.5 * t, t) for t in self.times]
        self.joint_trajectory.set_setpoints(new_setpoints)
        # The last setpoint should be at self.duration due to self.enforce_limits().
        # x and v should also be self.duration due to definition above.
        last_setpoint = ModifiableSetpoint(self.duration, 0.5 * self.duration, self.duration)
        self.assertEqual(self.joint_trajectory.setpoints[-1], last_setpoint)

    # duration.setter test
    def test_duration_setter(self):
        self.joint_trajectory.duration = self.duration * 2
        self.assertEqual(self.joint_trajectory.setpoints[-1].time, self.duration * 2)

    # get_interpolated_position tests
    def test_get_interpolated_position_below_zero(self):
        interpolated_position = self.joint_trajectory.get_interpolated_position(-1)
        self.assertEqual(interpolated_position, self.joint_trajectory.setpoints[0].position)

    def test_get_interpolated_position_above_duration(self):
        interpolated_position = self.joint_trajectory.get_interpolated_position(self.duration + 1)
        self.assertEqual(interpolated_position, self.joint_trajectory.setpoints[-1].position)

    def test_get_interpolated_position_middle(self):
        interpolated_position = self.joint_trajectory.get_interpolated_position(self.duration * 0.5)
        self.assertTrue(abs(interpolated_position - self.joint_trajectory.setpoints[1].position) < 0.02)

    # add_setpoint tests
    def test_add_setpoint_existence_new_setpoint(self):
        new_setpoint = ModifiableSetpoint(0.5, self.limits.upper + 1, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        new_setpoint_limited = ModifiableSetpoint(0.5, self.limits.upper, 0)
        self.assertEqual(self.joint_trajectory.setpoints[1], new_setpoint_limited)

    def test_add_setpoint_number_of_setpoints(self):
        new_setpoint = ModifiableSetpoint(0.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        self.assertEqual(len(self.joint_trajectory.setpoints), 4)

    def test_add_setpoint_save_changed_joints_call(self):
        new_setpoint = ModifiableSetpoint(0.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        self.gait_generator.save_changed_joints.assert_called_once_with([self.joint_trajectory])








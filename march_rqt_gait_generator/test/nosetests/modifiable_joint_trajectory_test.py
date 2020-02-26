import copy
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
        self.joint_trajectory = ModifiableJointTrajectory(self.joint_name, self.limits, copy.deepcopy(self.setpoints),
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

    # remove_setpoint tests
    def test_remove_setpoint_removal_correct_setpoint(self):
        self.joint_trajectory.remove_setpoint(0)
        new_first_setpoint = ModifiableSetpoint(0, 0.5, 1)
        self.assertEqual(self.joint_trajectory.setpoints[0], new_first_setpoint)

    def test_remove_setpoint_number_of_setpoints(self):
        self.joint_trajectory.remove_setpoint(1)
        self.assertEqual(len(self.joint_trajectory.setpoints), 2)

    def test_remove_setpoint_save_changed_joints_call(self):
        self.joint_trajectory.remove_setpoint(2)
        self.gait_generator.save_changed_joints.assert_called_once_with([self.joint_trajectory])

    # save_setpoints test
    def test_save_setpoints_content(self):
        self.joint_trajectory.save_setpoints()
        old_setpoints = self.joint_trajectory.setpoints_history[0]
        self.assertEqual(old_setpoints, self.setpoints)

    def test_save_setpoints_save_changed_joints_call(self):
        self.joint_trajectory.save_setpoints()
        self.gait_generator.save_changed_joints.assert_called_once_with([self.joint_trajectory])

    def test_save_setpoints_no_save_changed_joints_call(self):
        self.joint_trajectory.save_setpoints(single_joint_change=False)
        self.gait_generator.save_changed_joints.assert_not_called()

    # invert tests
    def test_invert_test_symmetric_times(self):
        self.joint_trajectory.invert()
        self.setpoints.reverse()
        for setpoint in self.setpoints:
            setpoint.velocity = - setpoint.velocity
            setpoint.time = self.duration - setpoint.time
        self.assertEqual(self.joint_trajectory.setpoints, self.setpoints)

    def test_invert_test_setpoints_saved(self):
        self.joint_trajectory.invert()
        self.assertEqual(self.joint_trajectory.setpoints_history[0], self.setpoints)
        self.gait_generator.save_changed_joints.assert_not_called()

    # undo tests
    def test_undo_empty_history(self):
        self.joint_trajectory.undo()
        self.assertEqual(self.joint_trajectory.setpoints, self.setpoints)

    def test_undo_add_setpoint(self):
        new_setpoint = ModifiableSetpoint(0.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        self.joint_trajectory.undo()
        self.assertEqual(self.joint_trajectory.setpoints, self.setpoints)

    def test_undo_multiple_things(self):
        new_setpoint = ModifiableSetpoint(0.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        setpoints_copy = copy.deepcopy(self.joint_trajectory.setpoints)
        new_setpoint = ModifiableSetpoint(0.7, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        self.joint_trajectory.invert()
        self.joint_trajectory.remove_setpoint(2)
        self.joint_trajectory.remove_setpoint(2)
        self.joint_trajectory.undo()
        self.joint_trajectory.undo()
        self.joint_trajectory.undo()
        self.joint_trajectory.undo()
        self.assertEqual(self.joint_trajectory.setpoints, setpoints_copy)

    def test_undo_max_history_memory(self):
        new_setpoint = ModifiableSetpoint(1.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        setpoints_copy = copy.deepcopy(self.joint_trajectory.setpoints)
        for i in range(100):
            new_setpoint = ModifiableSetpoint((i + 1) / 101.0, 0, 0)
            self.joint_trajectory.add_setpoint(new_setpoint)

        for i in range(110):
            self.joint_trajectory.undo()
        self.assertEqual(self.joint_trajectory.setpoints, setpoints_copy)

    def test_undo_max_history_memory_redo_list_length(self):
        new_setpoint = ModifiableSetpoint(1.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        for i in range(100):
            new_setpoint = ModifiableSetpoint((i + 1) / 101.0, 0, 0)
            self.joint_trajectory.add_setpoint(new_setpoint)

        for i in range(110):
            self.joint_trajectory.undo()
        self.assertEqual(len(self.joint_trajectory.setpoints_redo_list), 100)

    def test_undo_history_length(self):
        for i in range(100):
            new_setpoint = ModifiableSetpoint((i + 1) / 101.0, 0, 0)
            self.joint_trajectory.add_setpoint(new_setpoint)

        for i in range(80):
            self.joint_trajectory.undo()
        self.assertEqual(len(self.joint_trajectory.setpoints_history), 20)

    def test_undo_redo_list_length(self):
        for i in range(100):
            new_setpoint = ModifiableSetpoint((i + 1) / 101.0, 0, 0)
            self.joint_trajectory.add_setpoint(new_setpoint)

        for i in range(80):
            self.joint_trajectory.undo()
        self.assertEqual(len(self.joint_trajectory.setpoints_redo_list), 80)

    # redo tests
    def test_redo_empty_redo_list(self):
        self.joint_trajectory.redo()
        self.assertEqual(self.joint_trajectory.setpoints, self.setpoints)

    def test_undo_redo_add_setpoint(self):
        new_setpoint = ModifiableSetpoint(0.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        setpoints_copy = copy.deepcopy(self.joint_trajectory.setpoints)
        self.joint_trajectory.undo()
        self.joint_trajectory.redo()
        self.assertEqual(self.joint_trajectory.setpoints, setpoints_copy)

    def test_undo_redo_multiple_things(self):
        new_setpoint = ModifiableSetpoint(0.5, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        new_setpoint = ModifiableSetpoint(0.7, 0, 0)
        self.joint_trajectory.add_setpoint(new_setpoint)
        self.joint_trajectory.invert()
        self.joint_trajectory.remove_setpoint(2)
        setpoints_copy = copy.deepcopy(self.joint_trajectory.setpoints)
        self.joint_trajectory.remove_setpoint(2)
        self.joint_trajectory.undo()
        self.joint_trajectory.undo()
        self.joint_trajectory.undo()
        self.joint_trajectory.undo()
        self.joint_trajectory.redo()
        self.joint_trajectory.redo()
        self.joint_trajectory.redo()
        self.assertEqual(self.joint_trajectory.setpoints, setpoints_copy)

    def test_undo_redo_history_length(self):
        for i in range(100):
            new_setpoint = ModifiableSetpoint((i + 1) / 101.0, 0, 0)
            self.joint_trajectory.add_setpoint(new_setpoint)

        for i in range(80):
            self.joint_trajectory.undo()

        for i in range(50):
            self.joint_trajectory.redo()
        self.assertEqual(len(self.joint_trajectory.setpoints_history), 70)

    def test_undo_redo_redo_list_length(self):
        for i in range(100):
            new_setpoint = ModifiableSetpoint((i + 1) / 101.0, 0, 0)
            self.joint_trajectory.add_setpoint(new_setpoint)

        for i in range(80):
            self.joint_trajectory.undo()

        for i in range(50):
            self.joint_trajectory.redo()
        self.assertEqual(len(self.joint_trajectory.setpoints_redo_list), 30)

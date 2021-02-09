import unittest

from march_utility.exceptions.gait_exceptions import SubgaitInterpolationError
from march_utility.gait.joint_trajectory import JointTrajectory
from march_utility.gait.limits import Limits
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration


class JointTrajectoryTest(unittest.TestCase):
    def setUp(self):
        self.joint_name = "test_joint"
        self.limits = Limits(-1, 1, 2)
        self.duration = Duration(seconds=2.0)
        self.times = [Duration(), self.duration / 2.0, self.duration]
        self.setpoints = [
            Setpoint(t, 2 * t.seconds, t.seconds / 2.0) for t in self.times
        ]
        self.joint_trajectory = JointTrajectory(
            self.joint_name, self.limits, self.setpoints, self.duration
        )

    # get_setpoints_unzipped tests
    def test_get_setpoints_unzipped_time(self):
        output = self.joint_trajectory.get_setpoints_unzipped()
        self.assertEqual(
            output,
            (
                [t.seconds for t in self.times],
                [2 * t.seconds for t in self.times],
                [t.seconds / 2.0 for t in self.times],
            ),
        )

    # set_duration tests
    def test_set_duration(self):
        expected = Setpoint(
            self.joint_trajectory.setpoints[-1].time * 2,
            self.joint_trajectory.setpoints[-1].position,
            self.joint_trajectory.setpoints[-1].velocity / 2,
        )
        self.joint_trajectory.set_duration(self.duration * 2)
        self.assertEqual(self.joint_trajectory.setpoints[-1], expected)

    # validate_joint_transition() tests
    def test_valid_joint_transition(self):
        next_setpoints = [
            Setpoint(
                t, (self.duration - t).seconds * 2, (self.duration - t).seconds / 2.0
            )
            for t in self.times
        ]
        next_joint_trajectory = JointTrajectory(
            self.joint_name, self.limits, next_setpoints, self.duration
        )
        self.assertTrue(
            self.joint_trajectory.validate_joint_transition(next_joint_trajectory)
        )

    def test_invalid_joint_transition_position(self):
        next_setpoints = [
            Setpoint(t, (self.duration - t).seconds, (self.duration - t).seconds / 2.0)
            for t in self.times
        ]
        next_joint_trajectory = JointTrajectory(
            self.joint_name, self.limits, next_setpoints, self.duration
        )
        self.assertFalse(
            self.joint_trajectory.validate_joint_transition(next_joint_trajectory)
        )

    def test_invalid_joint_transition_velocity(self):
        next_setpoints = [
            Setpoint(t, 2 * (self.duration - t).seconds, (self.duration - t).seconds)
            for t in self.times
        ]
        next_joint_trajectory = JointTrajectory(
            self.joint_name, self.limits, next_setpoints, self.duration
        )
        self.assertFalse(
            self.joint_trajectory.validate_joint_transition(next_joint_trajectory)
        )

    # _validate_boundary_points tests
    def test_valid_boundary_points_nonzero_start_end_zero_speed(self):
        # First setpoint at t = 0.5 and last setpoint at t = 1.5 =/= duration have zero speed.
        setpoints = [
            Setpoint(
                t * 0.5 + Duration(seconds=0.5),
                (self.duration - t).seconds,
                t.seconds * 2 - t.seconds ** 2,
            )
            for t in self.times
        ]
        joint_trajectory = JointTrajectory(
            self.joint_name, self.limits, setpoints, self.duration
        )
        self.assertTrue(joint_trajectory._validate_boundary_points())

    def test_invalid_boundary_points_nonzero_start_nonzero_speed(self):
        # First setpoint at t = 1 has nonzero speed.
        setpoints = [
            Setpoint(
                t * 0.5 + Duration(seconds=1),
                (self.duration - t).seconds,
                (self.duration - t).seconds * 2,
            )
            for t in self.times
        ]
        joint_trajectory = JointTrajectory(
            self.joint_name, self.limits, setpoints, self.duration
        )
        self.assertFalse(joint_trajectory._validate_boundary_points())

    def test_invalid_boundary_points_nonzero_end_nonzero_speed(self):
        # Last setpoint at t = 1 =/= duration has nonzero speed.
        setpoints = [
            Setpoint(t * 0.5, (self.duration - t).seconds, t.seconds / 2.0)
            for t in self.times
        ]
        joint_trajectory = JointTrajectory(
            self.joint_name, self.limits, setpoints, self.duration
        )
        self.assertFalse(joint_trajectory._validate_boundary_points())

    # interpolate_setpoints tests
    def test_interpolation_start_point(self):
        interpolated_setpoint = self.joint_trajectory.get_interpolated_setpoint(
            Duration(seconds=0)
        )
        self.assertEqual(interpolated_setpoint, self.setpoints[0])

    def test_interpolation_end_point(self):
        interpolated_setpoint = self.joint_trajectory.get_interpolated_setpoint(
            self.duration
        )
        self.assertEqual(interpolated_setpoint, self.setpoints[-1])

    def test_interpolation_mid_point_position(self):
        interpolated_setpoint = self.joint_trajectory.get_interpolated_setpoint(
            self.duration / 2
        )
        self.assertEqual(interpolated_setpoint, self.setpoints[1])

    def test_get_interpolated_setpoints_invalid_time_too_high(self):
        setpoint = self.joint_trajectory.get_interpolated_setpoint(
            self.duration + Duration(seconds=1)
        )
        self.assertEqual(
            setpoint,
            Setpoint(
                self.duration + Duration(seconds=1), self.setpoints[-1].position, 0
            ),
        )

    def test_get_interpolated_setpoints_invalid_time_too_low(self):
        setpoint = self.joint_trajectory.get_interpolated_setpoint(Duration(seconds=-1))
        self.assertEqual(
            setpoint, Setpoint(Duration(seconds=-1), self.setpoints[0].position, 0)
        )

    def test_get_interpolated_setpoints_home_subgait(self):
        self.joint_trajectory.setpoints = [Setpoint(Duration(seconds=3), 1, 1)]
        setpoint = self.joint_trajectory.get_interpolated_setpoint(Duration(seconds=1))
        self.assertEqual(setpoint, Setpoint(Duration(seconds=1), 1, 1))

    def test_interpolate_trajectories_unequal_limits(self):
        different_limits = Limits(-2, 3, 2)
        other_trajectory = JointTrajectory(
            self.joint_name, different_limits, self.setpoints, self.duration
        )
        with self.assertRaises(SubgaitInterpolationError):
            JointTrajectory.interpolate_joint_trajectories(
                self.joint_trajectory, other_trajectory, 0.5
            )

    def test_interpolate_trajectories_unequal_amount_setpoints(self):
        other_trajectory = JointTrajectory(
            self.joint_name,
            self.limits,
            [self.setpoints[0], self.setpoints[-1]],
            self.duration,
        )
        with self.assertRaises(SubgaitInterpolationError):
            JointTrajectory.interpolate_joint_trajectories(
                self.joint_trajectory, other_trajectory, 0.5
            )

    def test_interpolate_trajectories_correct_duration(self):
        parameter = 0.5
        other_duration = self.duration + Duration(seconds=1)
        other_times = [Duration(), other_duration / 2.0, other_duration]
        other_setpoints = [
            Setpoint(t, 2 * t.seconds, t.seconds / 2.0) for t in other_times
        ]
        other_trajectory = JointTrajectory(
            self.joint_name, self.limits, other_setpoints, other_duration
        )
        new_trajectory = JointTrajectory.interpolate_joint_trajectories(
            self.joint_trajectory, other_trajectory, parameter
        )
        self.assertEqual(
            new_trajectory.duration,
            self.duration.weighted_average(other_duration, parameter),
        )

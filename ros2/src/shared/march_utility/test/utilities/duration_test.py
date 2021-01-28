import unittest
from copy import deepcopy

from march_utility.utilities.duration import Duration
from rclpy.duration import Duration as ROSDuration


class TestTransitionTrajectory(unittest.TestCase):
    def setUp(self):
        pass

    def test_get_nanoseconds(self):
        duration = Duration(seconds=1, nanoseconds=500000000)
        self.assertEqual(duration.nanoseconds, 1500000000)

    def test_get_seconds_from_nanoseconds(self):
        duration = Duration(nanoseconds=1000000000)
        self.assertAlmostEqual(duration.seconds, 1)

    def test_get_seconds(self):
        duration = Duration(seconds=1.5)
        self.assertAlmostEqual(duration.seconds, 1.5)

    def test_add(self):
        duration_1 = Duration(seconds=1)
        duration_2 = Duration(seconds=3)
        self.assertEqual((duration_1 + duration_2).nanoseconds, 4000000000)

    def test_sub(self):
        duration_1 = Duration(seconds=3)
        duration_2 = Duration(seconds=1)
        self.assertEqual((duration_1 - duration_2).nanoseconds, 2000000000)

    def test_mul(self):
        duration = Duration(seconds=1)
        self.assertEqual((duration * 7).nanoseconds, 7000000000)

    def test_truediv_numeric(self):
        duration = Duration(seconds=4)
        self.assertEqual((duration / 2).nanoseconds, 2000000000)

    def test_truediv_duration(self):
        duration_1 = Duration(seconds=4)
        duration_2 = Duration(seconds=1)
        self.assertEqual(duration_1 / duration_2, 4)

    def test_weighted_average(self):
        duration_1 = Duration(seconds=4)
        duration_2 = Duration(seconds=8)
        self.assertEqual(
            duration_1.weighted_average(duration_2, 0.75).nanoseconds, 7000000000
        )

    def test_round(self):
        duration = Duration(seconds=4.56)
        self.assertEqual(round(duration, 1).nanoseconds, 4600000000)

    def test_round_2(self):
        duration = Duration(seconds=2.16982)
        self.assertEqual(round(duration, 3).nanoseconds, 2170000000)

    def test_round_3(self):
        duration = Duration(nanoseconds=123456789)
        self.assertEqual(round(duration, 15).nanoseconds, 123456789)

    def test_from_ros_duration(self):
        duration = ROSDuration(seconds=3)
        self.assertEqual(Duration.from_ros_duration(duration).nanoseconds, 3000000000)

    def test_deepcopy(self):
        duration = Duration(seconds=1)
        self.assertEqual(deepcopy(duration).nanoseconds, 1000000000)

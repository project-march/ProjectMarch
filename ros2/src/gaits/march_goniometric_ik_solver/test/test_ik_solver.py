"""Author: Marten Haitjema, MVII"""

import unittest
import numpy as np

from march_goniometric_ik_solver.ik_solver import (
    Pose,
    LENGTH_LEG,
)


class TestIkSolver(unittest.TestCase):
    def setUp(self):
        self.pose = Pose()

    def test_calculate_joint_positions_zero_pose(self) -> None:
        self.pose.reset_to_zero_pose()
        ankle1_x = self.pose.calculate_joint_positions("pos_ankle1")[0]
        self.assertEqual(
            ankle1_x,
            0.0,
            "Ankle1 is not at 0.0 when in zero pose.",
        )
        hip_y = self.pose.calculate_joint_positions("pos_hip")[1]
        self.assertEqual(
            hip_y,
            LENGTH_LEG,
            "Hip_y is not equal to total leg length in zero pose.",
        )

    def test_calculate_joint_position_nonzero_pose(self) -> None:
        self.pose.fe_hip2 = np.pi / 2
        ankle2 = self.pose.calculate_joint_positions("pos_ankle2")
        ankle2_x = ankle2[0]
        self.assertEqual(
            ankle2_x,
            LENGTH_LEG,
            "At fe_hip2 angle of 90 deg, ankle2_x is not equal to total leg length",
        )
        ankle2_y = ankle2[1]
        self.assertEqual(
            ankle2_y,
            LENGTH_LEG,
            "At fe_hip2 angle of 90 deg, ankle2_y is not equal to total leg length",
        )

    def test_get_ankle_distance(self) -> None:
        self.pose.reset_to_zero_pose()
        ankle_distance = self.pose.get_ankle_distance()
        self.assertAlmostEqual(
            ankle_distance,
            0,
            4,
            "Distance between both ankles not equal to zero when in zero pose.",
        )
        self.pose.fe_hip1 = np.pi
        ankle_distance = self.pose.get_ankle_distance()
        self.assertAlmostEqual(
            ankle_distance,
            2 * LENGTH_LEG,
            4,
            "Distance between both ankles not equal to two times the leg length when fe_hip1 = pi",
        )
        self.pose.reset_to_zero_pose()
        self.pose.fe_hip2 = np.pi / 2
        ankle_distance = self.pose.get_ankle_distance()
        ankle_distance_pythagoras = np.sqrt(LENGTH_LEG ** 2 + LENGTH_LEG ** 2)
        self.assertEqual(
            ankle_distance,
            ankle_distance_pythagoras,
            "Distance between ankles not equal to length gotten using Pythagoras for hip angle of 45 deg.",
        )

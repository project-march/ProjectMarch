"""Author: Marten Haitjema, MVII"""

import unittest
import numpy as np

from march_goniometric_ik_solver.ik_solver import (
    Pose,
    DEFAULT_KNEE_BEND,
)


class TestIkSolver(unittest.TestCase):
    def setUp(self):
        self.pose = Pose()

    def test_calculate_joint_positions_zero_pose(self) -> None:
        self.pose.reset_to_zero_pose()
        ankle1_x = self.pose.calculate_joint_positions("pos_ankle1")[0]
        leg_length = self.pose.max_leg_length
        self.assertAlmostEqual(
            ankle1_x,
            0.0,
            2,
            "Ankle1 is not at 0.0 when in zero pose.",
        )
        hip_y = self.pose.calculate_joint_positions("pos_hip")[1]
        self.assertEqual(
            hip_y,
            leg_length,
            "Hip_y is not equal to total leg length in zero pose.",
        )

    def test_calculate_joint_position_nonzero_pose(self) -> None:
        try:
            knee_bend = self.pose.knee_bend
        except AttributeError:
            knee_bend = DEFAULT_KNEE_BEND
        self.pose.fe_hip2 = np.pi / 2 + knee_bend / 2
        ankle2 = self.pose.calculate_joint_positions("pos_ankle2")
        ankle2_x = ankle2[0]
        leg_length = self.pose.max_leg_length
        self.assertAlmostEqual(
            ankle2_x,
            leg_length,
            2,
            "At fe_hip2 angle of 90 deg, ankle2_x is not equal to total leg length",
        )
        ankle2_y = ankle2[1]
        self.assertAlmostEqual(
            ankle2_y,
            leg_length,
            2,
            "At fe_hip2 angle of 90 deg, ankle2_y is not equal to total leg length",
        )

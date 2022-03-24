"""Author: Marten Haitjema, MVII"""

import unittest
import numpy as np

from march_goniometric_ik_solver.quadrilateral_angle_solver import (
    get_angle_between_points,
    find_fourth_point,
    get_angles,
)


class TestQuadrilateralAngleSolver(unittest.TestCase):
    def test_get_angle_between_points_right_angled_triangle(self) -> None:
        # right angle
        a = np.array([0, 1])
        b = np.array([0, 0])
        c = np.array([1, 0])
        points = [a, b, c]
        angle = np.pi / 2

        self.assertEqual(
            angle,
            get_angle_between_points(points),
            "Angle between points is not correct for a right_anlged triangle",
        )

    def test_get_angle_between_points_negative_right_angled_triangle(self) -> None:
        a = np.array([0, -1])
        b = np.array([0, 0])
        c = np.array([-1, 0])
        points = [a, b, c]
        angle = np.pi / 2

        self.assertEqual(
            angle,
            get_angle_between_points(points),
            "Angle between points is not correct for a right_anlged triangle",
        )

    def test_get_angle_between_point_sharp_angle(self) -> None:
        a = np.array([0, 1])
        b = np.array([0, 0])
        c = np.dot(np.array([1, 0]), np.array([np.cos(np.pi / 4), np.sin(np.pi / 4)]))
        points = [a, b, c]
        angle = np.pi / 4

        self.assertEqual(
            angle,
            get_angle_between_points(points),
            "Angle between points is not correct for a 45 deg triangle",
        )

    def test_find_fourth_point_unit_square(self) -> None:
        a = np.array([0, 0])
        b = np.array([1, 0])
        c = np.array([1, 1])
        da = -1
        cd = -1
        convex = True

        d_expected = np.array([0, 1])
        d_actual = find_fourth_point(a, b, c, da, cd, convex)

        self.assertAlmostEqual(
            d_expected[0],
            d_actual[0],
            4,
            "Fourth point x-coordinate not correct based on unit square",
        )
        self.assertAlmostEqual(
            d_expected[1],
            d_actual[1],
            4,
            "Fourth point y-coordinate not correct based on unit square",
        )

    def test_get_angles_unit_square(self) -> None:
        a = np.array([0, 0])
        b = np.array([1, 0])
        c = np.array([1, 1])
        d = np.array([0, 1])
        points = [a, b, c, d]

        angles_expected = [np.pi / 2] * 4
        angles_actual = get_angles(points)

        # for loop because assertAlmostEqual does not exist for lists
        self.assertListEqual(
            angles_expected,
            angles_actual,
            "Angles not equal to 90 degrees for square",
        )

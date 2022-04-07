import numpy as np
from typing import List


def get_angle_from_sides(opposite_side: float, adjacent_sides: List[float]) -> float:
    """Calculates the angle in a triangle opposite to 'opposite_side' when all sides are given.

    Based on the cosine rule.

    Args:
        opposite_side (float): the length of the side opposite to the angle we like to know.
        adjacent_sides (List[float]): the lengths of the adjacent sides to the angle we like to know.

    Returns:
        float: the angle calculated in rad.
    """

    return np.arccos((adjacent_sides.dot(adjacent_sides) - opposite_side ** 2) / (2 * np.prod(adjacent_sides)))


def get_angles_from_sides(sides: List[float]) -> List[float]:
    """Calculates all the angles in a triangle when all sides are given.

    Args:
        sides (List[float]): a list containing the lengths of the sides of a triangle.

    Returns:
        List[float]: the angles of the triangle in order of opposite to sides given.
    """

    angles = []

    for i in range(len(sides)):
        opposite_side = sides[i]
        adjacent_sides = np.delete(sides, i)

        angle = get_angle_from_sides(opposite_side, adjacent_sides)
        angles.append(angle)

    return angles

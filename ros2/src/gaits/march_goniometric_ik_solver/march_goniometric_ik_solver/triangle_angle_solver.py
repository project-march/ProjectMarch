import numpy as np


def get_angle_from_sides(opposite_side: float, adjacent_sides: list):
    """
    Calculates the angle in a triangle opposite to 'opposite_side'
    when all sides are given, based on the cosine rule.
    """

    return np.arccos(
        (adjacent_sides.dot(adjacent_sides) - opposite_side ** 2)
        / (2 * np.prod(adjacent_sides))
    )


def get_angles_from_sides(sides: list):
    """
    Calculates all the angles in a triangle when all sides are given.
    Angles are returned in the order of opposite sides.
    """

    angles = []

    for i in range(len(sides)):
        opposite_side = sides[i]
        adjacent_sides = np.delete(sides, i)

        angle = get_angle_from_sides(opposite_side, adjacent_sides)
        angles.append(angle)

    return angles

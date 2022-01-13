import numpy as np
from typing import List

RIGHT_ANGLE = np.pi / 2


def get_angle_between_points(points: List[np.array]) -> float:
    """
    Calculates the angle between three points,
    where the angle is calculated for the middle point.
    Based on the dot product cosine rule.
    """

    a = points[0]
    b = points[1]
    c = points[2]

    ba = a - b
    bc = c - b
    return np.arccos(np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc)))


def find_fourth_point(
    a: np.array,
    b: np.array,
    c: np.array,
    da: float,
    cd: float,
    convex: bool,
) -> np.array:
    """
    Finds the fourth point (d) of a quadrilateral when 3 points (a, b, c) and the
    distances da and cd are given, based on
    http://paulbourke.net/geometry/circlesphere/#:~:text=Intersection%20of%20two%20circles
    """

    r0, r1 = da, cd
    p0, p1 = a, c
    d = np.linalg.norm(p0 - p1)

    dis_p0_p2 = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
    h = np.sqrt(r0 ** 2 - dis_p0_p2 ** 2)
    p2 = p0 + dis_p0_p2 * (p1 - p0) / d

    kernel = np.array([-1, 1]) if convex else np.array([1, -1])

    return p2 + kernel * h / d * np.flip(p1 - p0)  # = p3


def get_angles(points: List[np.array]) -> List[float]:
    """
    Calculates the angles of a quadrilateral by giving the four points of it.
    The order of returned angles is equal to the order of points given.
    """

    angles = []

    for i in range(len(points)):
        # Usually the point i and the previous (i-1) and next (i+1) points are used:
        if i < len(points) - 1:
            angle = get_angle_between_points([points[i - 1], points[i], points[i + 1]])
        # But for the last angle, there is no (i_+1), so points[0] is used:
        else:
            angle = get_angle_between_points([points[i - 1], points[i], points[0]])
        angles.append(angle)
    return angles


def solve_quadritlateral(
    lengths: List[float], angle_b: float, convex: bool = True, debug: bool = False
) -> List[float]:
    """
    Calculates the angles of a quadrilateral given all side lengths and the angle of point b.
    Expects lengths to be given as [da, ab, bc, cd]
    It first determines the location of all points where point a is at (0,0).
    Next it calculates and returns all angles.
    """

    da, ab, bc, cd = lengths

    a = np.array([0, 0])
    b = a + np.array([ab, 0])
    c = b + np.array([-np.cos(angle_b) * bc, np.sin(angle_b) * bc])
    d = find_fourth_point(a, b, c, da, cd, convex)
    points = [a, b, c, d]

    angles = get_angles(points)

    if debug:
        print("Angles are: ", angles)
        check_lengths(points, lengths)

    return angles


def check_lengths(points: List[float], real_lengths: List[float]):
    """
    Checks whether the lengths between points are equal to the real lengths.
    Expects points as [a, b, c, d] and real_lengths as [da, ab, bc, cd].
    This method is only used for debugging.
    """

    for i in range(len(points)):
        length = np.linalg.norm(points[i - 1] - points[i])
        error = abs(length - real_lengths[i])
        if error > 1e-10:
            print("Error difference = ", error)

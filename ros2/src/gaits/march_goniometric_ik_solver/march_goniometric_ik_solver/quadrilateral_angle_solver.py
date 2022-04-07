import numpy as np
from typing import List

RIGHT_ANGLE = np.pi / 2


def get_angle_between_points(points: List[np.array]) -> float:
    """Calculates the angle between three points.

    The angle is calculated for the middle point.
    Based on the dot product cosine rule.

    Args:
        points (List[np.array]): a list of 3 points, where each point is a 2D array containing the location of the point.

    Returns:
        float: the angle between the three points.
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
    """Finds the fourth point (d) of a quadrilateral when 3 points (a, b, c) and the distances da and cd are given.

    Based on http://paulbourke.net/geometry/circlesphere/#:~:text=Intersection%20of%20two%20circles.

    Args:
        a (np.array): a 2D array of the position of the first point of the quadrilateral.
        b (np.array): a 2D array of the position of the second point of the quadrilateral.
        c (np.array): a 2D array of the position of the third point of the quadrilateral.
        da (float): the distance between point a and the fourth point (d) we try to find.
        cd (float): the distance between point c and the fourth point (d) we try to find.
        convex (bool): whether the quadrilateral is convex or not.

    Returns:
        np.array: the location of the fourth point of the quadrilateral.
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
    """Calculates the angles of a quadrilateral by giving the four points of it.

    Args:
        points (List[np.array]): A list containing the positions (2D numpy arrays) of the four points of a quadrilateral.

    Returns:
        List[float]: the four angles of the quadrilateral in the same order as the four points are given.
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


def solve_quadritlateral(lengths: List[float], angle_b: float, convex: bool = True, debug: bool = False) -> List[float]:
    """Calculates the angles of a quadrilateral given all side lengths and the angle of point b.

    It first determines the location of all points where point a is at (0,0).
    Next it calculates and returns all angles.

    Args:
        lengths (List[float]): the lengths of the four sides of a quadrilateral, with expected order as [da, ab, bc, cd].
        angle_b (float): the angle in the quadrilateral at point b.
        convex (bool): whether the quadrilateral is convex or not.
        debug (bool): whether debug mode is enabled or not.

    Returns:
        List[float]: the four angles of the quadrilateral.
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
    """Checks whether the calculated lengths between points are equal to the real lengths.

    Expects points as [a, b, c, d] and real_lengths as [da, ab, bc, cd].
    This method is only used for debugging.

    Args:
        points (List[np.array]): A list containing the positions (2D numpy arrays) of the four points [a, b, c, d] of a quadrilateral.
        real_lengths (List[float]): A list containing the real lengths [da, ab, bc, cd] of a quadrilateral.
    """

    for i in range(len(points)):
        length = np.linalg.norm(points[i - 1] - points[i])
        error = abs(length - real_lengths[i])
        if error > 1e-10:
            print("Error difference = ", error)

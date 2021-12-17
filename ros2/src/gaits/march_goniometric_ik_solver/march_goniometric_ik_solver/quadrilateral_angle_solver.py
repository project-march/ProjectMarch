import numpy as np
import matplotlib.pyplot as plt

RIGHT_ANGLE = np.pi / 2


def plot_points(points):
    plt.figure(1)
    for i in points:
        plt.plot(i[0], i[1], marker="o")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()


def get_angle_between_points(points):
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


def find_fourth_point(a, b, c, da):
    """
    Finds the fourth point (d) of a quadrilateral when 3 points (a, b, c) and the
    distance da or cd is given, assuming da = cd, with the following step:
    1. Calculate the center_point between a and c.
    2. Calculate the distance between a and the center_point, which is half of the diagonal ac.
    3. Calculate the distance between point d and the diagonal ac, which we call height, using pythagoras theorem.
    4. Determine the angle between ab and ac, which we call ground_angle.
    5. Determine the direction of the height calculated in step 3, by knowing it is perpenicular to ac and assuming a convex quadrilateral.
    6. Calculate point d by adding the height as vector with correct direction to center_point, and return it.
    """

    center_point = np.array([np.mean([a[0], c[0]]), np.mean([a[1], c[1]])])
    half_ac = np.linalg.norm(a - c) / 2
    height = np.sqrt(da ** 2 - half_ac ** 2)

    ground_angle = get_angle_between_points([c, a, b])
    center_point_angle = RIGHT_ANGLE - ground_angle

    return center_point + np.array(
        [-np.cos(center_point_angle) * height, np.sin(center_point_angle) * height]
    )


def get_angles(points):
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


def check_lengths(points, real_lengths):
    """
    Checks whether the lengths between points are equal to the real lengths.
    Expects points as [a, b, c, d] and real_lengths as [da, ab, bc, cd]
    """

    for i in range(len(points)):
        length = np.linalg.norm(points[i - 1] - points[i])
        error = abs(length - real_lengths[i])
        if error > 1e-10:
            print("Error difference = ", error)


def solve_quadritlateral(lengths, angle_b, debug=False):
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
    d = find_fourth_point(a, b, c, da)
    points = [a, b, c, d]

    angles = get_angles(points)

    if debug:
        print("Angles are: ", angles)
        check_lengths(points, lengths)

    return angles

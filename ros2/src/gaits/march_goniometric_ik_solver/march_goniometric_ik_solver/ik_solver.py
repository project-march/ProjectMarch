import numpy as np
import matplotlib.pyplot as plt
import time

import march_goniometric_ik_solver.quadrilateral_angle_solver as qas
import march_goniometric_ik_solver.triangle_angle_solver as tas
from march_goniometric_ik_solver.goniometric_functions_degrees import (
    cos,
    sin,
    asin,
    atan,
)


# Constants:
length_upper_leg = 41  # cm
length_lower_leg = 41  # cm
length_foot = 10  # cm
length_leg = length_upper_leg + length_lower_leg
ankle_zero_angle = 90  # deg
knee_zero_angle = 180  # deg
default_max_ankle_flexion = 10  # deg


def calculate_joint_positions(pose, joint="all"):
    """
    Calculates the joint positions for a given pose.
    """

    # convert pose list into the individual joint flexions:
    flex_ankle1, flex_knee1, flex_hip1, flex_hip2, flex_knee2, flex_ankle2 = pose

    # ankle1 is defined at [0,0]:
    pos_ankle1 = np.array([0, 0])

    # assumed flat feet, resulting in toe1 at [length_foot, 0]:
    pos_toe1 = np.array([length_foot, 0])

    # knee1 = ankle1 + translation_by_lower_leg:
    pos_knee1 = np.array(
        [
            pos_ankle1[0] + sin(flex_ankle1) * length_lower_leg,
            pos_ankle1[1] + cos(flex_ankle1) * length_lower_leg,
        ]
    )

    # hip1 = knee1 + translation_by_upper_leg:
    pos_hip = np.array(
        [
            pos_knee1[0] + sin(flex_ankle1 - flex_knee1) * length_upper_leg,
            pos_knee1[1] + cos(flex_ankle1 - flex_knee1) * length_upper_leg,
        ]
    )

    # knee2 = hip + translation_by_upper_leg:
    pos_knee2 = np.array(
        [
            pos_hip[0] + sin(flex_hip2) * length_upper_leg,
            pos_hip[1] - cos(flex_hip2) * length_upper_leg,
        ]
    )

    # ankle2 = knee2 + translation_by_lower_leg:
    pos_ankle2 = np.array(
        [
            pos_knee2[0] + sin(flex_hip2 - flex_knee2) * length_lower_leg,
            pos_knee2[1] - cos(flex_hip2 - flex_knee2) * length_lower_leg,
        ]
    )

    # toe2 = ankle2 + translation_by_foot:
    angle_ankle2 = ankle_zero_angle + flex_ankle2
    pos_toe2 = np.array(
        [
            pos_ankle2[0] + sin(flex_hip2 - flex_knee2 + angle_ankle2) * length_foot,
            pos_ankle2[1] - cos(flex_hip2 - flex_knee2 + angle_ankle2) * length_foot,
        ]
    )

    # return all positions, or one specific if asked:
    if joint == "all":
        return pos_toe1, pos_ankle1, pos_knee1, pos_hip, pos_knee2, pos_ankle2, pos_toe2
    else:
        return locals()[joint]


def make_plot(pose):
    """
    Makes a plot of the exo by first calculating the joint positions
    and then plotting them with lines.
    """

    positions = calculate_joint_positions(pose)
    positions_x = [pos[0] for pos in positions]
    positions_y = [pos[1] for pos in positions]

    plt.figure(1)
    plt.plot(positions_x, positions_y)
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()


def calculate_ground_pose_flexion(ankle_x):
    """
    Calculates and returns the flexion of the ankles and the hips when the
    ankle is moved to a certain x position, using pythagoras theorem.
    """

    return asin((ankle_x / 2) / length_leg)


def calculate_lifted_pose(pos_ankle2, pose):
    """
    Calculate the pose after lifting the foot to the desired ankle postion.
    """

    # calculate angles using triangle between hip, knee2 and ankle2 and side distances:
    pos_hip = calculate_joint_positions(pose, "pos_hip")
    dist_hip_ankle = np.linalg.norm(pos_hip - pos_ankle2)
    sides = [length_lower_leg, dist_hip_ankle, length_upper_leg]
    angle_hip, angle_knee2, angle_ankle2 = tas.get_angles_from_sides(sides)

    # define new flex_hip2:
    point_below_hip = np.array([pos_ankle2[0] / 2, 0])
    hip_angle_vertical_ankle2 = qas.get_angle_between_points(
        [point_below_hip, pos_hip, pos_ankle2]
    )
    flex_hip2 = hip_angle_vertical_ankle2 + angle_hip

    # define new flex_knee2:
    flex_knee2 = knee_zero_angle - angle_knee2

    # define new flex_ankle2:
    toe2 = pos_ankle2 + np.array([length_foot, 0])
    ankle2_angle_toe2_hip = qas.get_angle_between_points([toe2, pos_ankle2, pos_hip])
    flex_ankle2 = ankle_zero_angle - (ankle2_angle_toe2_hip - angle_ankle2)

    pose[3:6] = flex_hip2, flex_knee2, flex_ankle2
    return pose


def reduce_dorsi_flexion(pose, max_flexion):
    """
    Calculate the pose after reducing the dorsiflexion using quadrilateral solver
    with quadrilateral between ankle2, knee2, hip, knee1
    """

    flex_knee1, flex_hip1, flex_hip2, flex_knee2, flex_ankle2 = pose[1:]

    # get current state:
    (
        pos_ankle1,
        pos_knee1,
        pos_hip,
        pos_knee2,
        pos_ankle2,
    ) = calculate_joint_positions(pose)[1:-1]

    # determine angle_ankle2 of quadrilateral:
    reduction = flex_ankle2 - max_flexion
    angle_ankle2 = (
        qas.get_angle_between_points([pos_knee1, pos_ankle2, pos_knee2]) - reduction
    )

    # determine other angles using angle_ankle2 and sides:
    dist_knee1_ankle2 = np.linalg.norm(pos_knee1 - pos_ankle2)
    sides = [
        length_upper_leg,
        dist_knee1_ankle2,
        length_lower_leg,
        length_upper_leg,
    ]
    angle_knee1, angle_ankle2, angle_knee2, angle_hip = qas.solve_quadritlateral(
        sides, angle_ankle2
    )

    # define new flex_knee1:
    knee1_angle_ankle1_ankle2 = qas.get_angle_between_points(
        [pos_ankle1, pos_knee1, pos_ankle2]
    )
    flex_knee1 = angle_knee1 + knee1_angle_ankle1_ankle2 - knee_zero_angle
    pose[1] = flex_knee1

    # get new hip location and determine point below it:
    pos_hip = calculate_joint_positions(pose, "pos_hip")
    point_below_hip = np.array([pos_hip[0], 0])

    # define new flex_hip1:
    if pos_hip[0] < pos_knee1[0]:
        flex_hip1 = qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])
    else:
        flex_hip1 = -qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

    # define new flex_hip2, flex_knee2 and flex_ankle2:
    flex_hip2 = angle_hip + flex_hip1
    flex_knee2 = knee_zero_angle - angle_knee2
    flex_ankle2 -= reduction

    pose[2:6] = (
        flex_hip1,
        flex_hip2,
        flex_knee2,
        flex_ankle2,
    )

    return pose


def straighten_leg(pose):
    """
    Straighten stance leg by making a triangle between ankle1, hip and knee2
    and calculating new angles.
    """

    # get current state:
    (
        pos_toe1,
        pos_ankle1,
        pos_knee1,
        pos_hip,
        pos_knee2,
        pos_ankle2,
        pos_toe2,
    ) = calculate_joint_positions(pose)

    # determine sides of triangle and calculate angles:
    dist_ankle1_knee2 = np.linalg.norm(pos_ankle1 - pos_knee2)
    sides = [length_upper_leg, length_leg, dist_ankle1_knee2]
    angle_ankle1, angle_knee2, angle_hip = tas.get_angles_from_sides(sides)

    # define new flex_ankle1 and flex_knee1:
    ankle1_angle_toe1_knee2 = qas.get_angle_between_points(
        [pos_knee2, pos_ankle1, pos_toe1]
    )
    flex_ankle1 = ankle_zero_angle - (angle_ankle1 + ankle1_angle_toe1_knee2)
    flex_knee1 = 0
    pose[0:2] = flex_ankle1, flex_knee1

    # get new knee1 and hip location and determine point below it:
    pos_knee1 = calculate_joint_positions(pose, "pos_knee1")
    pos_hip = calculate_joint_positions(pose, "pos_hip")
    point_below_hip = np.array([pos_hip[0], 0])

    # define new flex_hip1:
    if pos_hip[0] < pos_knee1[0]:
        flex_hip1 = qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])
    else:
        flex_hip1 = -qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

    # define new flex_hip2 and flex_knee2:
    flex_hip2 = angle_hip + flex_hip1
    knee2_angle_ankle1_ankle2 = qas.get_angle_between_points(
        [pos_ankle1, pos_knee2, pos_ankle2]
    )
    flex_knee2 = knee_zero_angle - (angle_knee2 + knee2_angle_ankle1_ankle2)

    pose[3:5] = flex_hip2, flex_knee2

    return pose


def solve_mid_position(
    ankle_x,
    ankle_y,
):
    """
    Solve inverse kinematics for the middle position. Assumes that the
    stance leg is straight. Takes the ankle_x and ankle_y position of the
    desired middle position. Calculates the required hfe and kfe angles of
    the swing leg by making a triangle between the swing leg ankle, swing leg
    knee and the hip. Returns the calculated pose.
    """

    hip_x = 0
    hip_y = length_leg

    # Calculate distance from ankle to hip with Pythagoras
    diff_x = ankle_x - hip_x
    diff_y = ankle_y - hip_y
    len_ankle_to_hip = np.sqrt((diff_x ** 2) + (diff_y ** 2))

    swing_leg_angles = tas.get_angles_from_sides(
        [length_lower_leg, len_ankle_to_hip, length_upper_leg]
    )

    # The angle found with the triangle between the swing ankle/knee and the
    # hip is not the same as the hfe angle, which is the angle between the
    # upper leg of the stance leg and the upper leg of the swing leg.
    hfe_offset = atan(diff_x / diff_y)
    swing_leg_hfe = swing_leg_angles[0] - hfe_offset
    swing_leg_kfe = knee_zero_angle - swing_leg_angles[1]

    # HAA ankle is fixed at 1.72 deg (0.03 rad) for now
    pose = [0.0, 0.0, 0.0, 1.72, 1.72, swing_leg_hfe, swing_leg_kfe, 0.0]

    return [np.deg2rad(angle) for angle in pose]


def solve_end_position(
    ankle_x,
    ankle_y=0,
    max_ankle_flexion=default_max_ankle_flexion,
    plot=False,
    timer=False,
):
    """
    Solve inverse kinematics for a desired ankle location, assuming flat feet.
    Expects at least the ankle x-position and returns the calculated pose.
    """
    start = time.time()

    # calculate ground pose:
    ground_pose_flexion = calculate_ground_pose_flexion(ankle_x)
    pose = [
        ground_pose_flexion,
        0,
        -ground_pose_flexion,
        ground_pose_flexion,
        0,
        -ground_pose_flexion,
    ]

    # calculate lifted pose if ankle_y > 0:
    if ankle_y > 0:
        pos_ankle = np.array([ankle_x, ankle_y])
        pose = calculate_lifted_pose(pos_ankle, pose)

        # reduce dorsi flexion and straighten leg if flex_ankle2 > max_flexion:
        flex_ankle2 = pose[-1]
        if flex_ankle2 > max_ankle_flexion:

            # reduce dorsi flexion:
            pose = reduce_dorsi_flexion(pose, max_ankle_flexion)

            # straighten leg:
            pose = straighten_leg(pose)

    if timer:
        end = time.time()
        print("Calculation time = ", end - start, " seconds")

    if plot:
        make_plot(pose)

    pose.insert(3, np.rad2deg(0.03))
    pose.insert(4, np.rad2deg(0.03))

    return [np.deg2rad(angle) for angle in pose]

import numpy as np
import matplotlib.pyplot as plt
import time

import march_goniometric_ik_solver.quadrilateral_angle_solver as qas
import march_goniometric_ik_solver.triangle_angle_solver as tas
from march_goniometric_ik_solver.goniometric_functions_degrees import (
    cos,
    sin,
    asin,
)

from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
    get_limits_robot_from_urdf_for_inverse_kinematics,
)

# Get leg lengths form urdf:
(
    length_upper_leg,
    length_lower_leg,
) = get_lengths_robot_from_urdf_for_inverse_kinematics()[0:2]
length_leg = length_upper_leg + length_lower_leg

# Get ankle limit from urdf:
limits = get_limits_robot_from_urdf_for_inverse_kinematics("right_ankle")
default_max_ankle_flexion = np.rad2deg(limits.upper)

# Constants:
length_foot = 0.10  # m
ankle_zero_angle = 90  # deg
knee_zero_angle = 180  # deg

default_hip_aa = 1.72  # deg


class Pose:
    def __init__(self, pose):
        (
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
        ) = pose

    def calculate_joint_positions(self, joint="all"):
        """
        Calculates the joint positions for a given pose.
        """

        # ankle1 is defined at [0,0]:
        pos_ankle1 = np.array([0, 0])

        # assumed flat feet, resulting in toe1 at [length_foot, 0]:
        pos_toe1 = np.array([length_foot, 0])

        # knee1 = ankle1 + translation_by_lower_leg:
        pos_knee1 = np.array(
            [
                pos_ankle1[0] + sin(self.fe_ankle1) * length_lower_leg,
                pos_ankle1[1] + cos(self.fe_ankle1) * length_lower_leg,
            ]
        )

        # hip1 = knee1 + translation_by_upper_leg:
        pos_hip = np.array(
            [
                pos_knee1[0] + sin(self.fe_ankle1 - self.fe_knee1) * length_upper_leg,
                pos_knee1[1] + cos(self.fe_ankle1 - self.fe_knee1) * length_upper_leg,
            ]
        )

        # knee2 = hip + translation_by_upper_leg:
        pos_knee2 = np.array(
            [
                pos_hip[0] + sin(self.fe_hip2) * length_upper_leg,
                pos_hip[1] - cos(self.fe_hip2) * length_upper_leg,
            ]
        )

        # ankle2 = knee2 + translation_by_lower_leg:
        pos_ankle2 = np.array(
            [
                pos_knee2[0] + sin(self.fe_hip2 - self.fe_knee2) * length_lower_leg,
                pos_knee2[1] - cos(self.fe_hip2 - self.fe_knee2) * length_lower_leg,
            ]
        )

        # toe2 = ankle2 + translation_by_foot:
        angle_ankle2 = ankle_zero_angle + self.fe_ankle2
        pos_toe2 = np.array(
            [
                pos_ankle2[0]
                + sin(self.fe_hip2 - self.fe_knee2 + angle_ankle2) * length_foot,
                pos_ankle2[1]
                - cos(self.fe_hip2 - self.fe_knee2 + angle_ankle2) * length_foot,
            ]
        )

        # return all positions, or one specific if asked:
        if joint == "all":
            return (
                pos_toe1,
                pos_ankle1,
                pos_knee1,
                pos_hip,
                pos_knee2,
                pos_ankle2,
                pos_toe2,
            )
        else:
            return locals()[joint]

    def get_pose(self):
        return (
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
        )

    def make_plot(self):
        """
        Makes a plot of the exo by first calculating the joint positions
        and then plotting them with lines.
        """

        positions = self.calculate_joint_positions()
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


def calculate_lifted_pose(pos_ankle2, pose: Pose):
    """
    Calculate the pose after lifting the foot to the desired ankle postion.
    """

    # calculate angles using triangle between hip, knee2 and ankle2 and side distances:
    pos_hip = pose.calculate_joint_positions("pos_hip")
    dist_hip_ankle = np.linalg.norm(pos_hip - pos_ankle2)
    sides = [length_lower_leg, dist_hip_ankle, length_upper_leg]
    angle_hip, angle_knee2, angle_ankle2 = tas.get_angles_from_sides(sides)

    # define new fe_hip2:
    point_below_hip = np.array([pos_ankle2[0] / 2, 0])
    hip_angle_vertical_ankle2 = qas.get_angle_between_points(
        [point_below_hip, pos_hip, pos_ankle2]
    )
    fe_hip2 = hip_angle_vertical_ankle2 + angle_hip

    # define new fe_knee2:
    fe_knee2 = knee_zero_angle - angle_knee2

    # define new fe_ankle2:
    toe2 = pos_ankle2 + np.array([length_foot, 0])
    ankle2_angle_toe2_hip = qas.get_angle_between_points([toe2, pos_ankle2, pos_hip])
    fe_ankle2 = ankle_zero_angle - (ankle2_angle_toe2_hip - angle_ankle2)

    pose.fe_hip2, pose.fe_knee2, pose.fe_ankle2 = (
        fe_hip2,
        fe_knee2,
        fe_ankle2,
    )
    return pose


def reduce_dorsi_flexion(max_flexion, pose: Pose):
    """
    Calculate the pose after reducing the dorsiflexion using quadrilateral solver
    with quadrilateral between ankle2, knee2, hip, knee1
    """

    fe_knee1, fe_hip1, fe_hip2, fe_knee2, fe_ankle2 = (
        pose.fe_knee1,
        pose.fe_hip1,
        pose.fe_hip2,
        pose.fe_knee2,
        pose.fe_ankle2,
    )

    # get current state:
    (
        pos_ankle1,
        pos_knee1,
        pos_hip,
        pos_knee2,
        pos_ankle2,
    ) = pose.calculate_joint_positions()[1:-1]

    # determine angle_ankle2 of quadrilateral:
    reduction = fe_ankle2 - max_flexion
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

    # define new fe_knee1:
    knee1_angle_ankle1_ankle2 = qas.get_angle_between_points(
        [pos_ankle1, pos_knee1, pos_ankle2]
    )
    fe_knee1 = angle_knee1 + knee1_angle_ankle1_ankle2 - knee_zero_angle
    pose.fe_knee1 = fe_knee1

    # get new hip location and determine point below it:
    pos_hip = pose.calculate_joint_positions("pos_hip")
    point_below_hip = np.array([pos_hip[0], 0])

    # define new fe_hip1:
    if pos_hip[0] < pos_knee1[0]:
        fe_hip1 = qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])
    else:
        fe_hip1 = -qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

    # define new fe_hip2, fe_knee2 and fe_ankle2:
    fe_hip2 = angle_hip + fe_hip1
    fe_knee2 = knee_zero_angle - angle_knee2
    fe_ankle2 -= reduction

    pose.fe_hip1 = fe_hip1
    pose.fe_hip2 = fe_hip2
    pose.fe_knee2 = fe_knee2
    pose.fe_ankle2 = fe_ankle2

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
    ) = pose.calculate_joint_positions()

    # determine sides of triangle and calculate angles:
    dist_ankle1_knee2 = np.linalg.norm(pos_ankle1 - pos_knee2)
    sides = [length_upper_leg, length_leg, dist_ankle1_knee2]
    angle_ankle1, angle_knee2, angle_hip = tas.get_angles_from_sides(sides)

    # define new fe_ankle1 and fe_knee1:
    ankle1_angle_toe1_knee2 = qas.get_angle_between_points(
        [pos_knee2, pos_ankle1, pos_toe1]
    )
    fe_ankle1 = ankle_zero_angle - (angle_ankle1 + ankle1_angle_toe1_knee2)
    fe_knee1 = 0
    pose.fe_ankle1 = fe_ankle1
    pose.fe_knee1 = fe_knee1

    # get new knee1 and hip location and determine point below it:
    pos_knee1 = pose.calculate_joint_positions("pos_knee1")
    pos_hip = pose.calculate_joint_positions("pos_hip")
    point_below_hip = np.array([pos_hip[0], 0])

    # define new fe_hip1:
    if pos_hip[0] < pos_knee1[0]:
        fe_hip1 = qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])
    else:
        fe_hip1 = -qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

    # define new fe_hip2 and fe_knee2:
    fe_hip2 = angle_hip + fe_hip1
    knee2_angle_ankle1_ankle2 = qas.get_angle_between_points(
        [pos_ankle1, pos_knee2, pos_ankle2]
    )
    fe_knee2 = knee_zero_angle - (angle_knee2 + knee2_angle_ankle1_ankle2)

    pose.fe_hip2 = fe_hip2
    pose.fe_knee2 = fe_knee2

    return pose


def solve_mid_position(ankle_x, ankle_y, plot=False):
    """
    Solve inverse kinematics for the middle position. Assumes that the
    stance leg is straight. Takes the ankle_x and ankle_y position of the
    desired middle position. Calculates the required hip and knee angles of
    the swing leg by making a triangle between the swing leg ankle, swing leg
    knee and the hip. Returns the calculated pose.
    """

    ankle2 = np.array([ankle_x, ankle_y])
    hip = np.array([0, length_leg])
    dist_ankle_hip = np.linalg.norm(ankle2 - hip)

    # Calculate hip and knee2 angle in triangle with ankle2:
    angle_hip, angle_knee2 = tas.get_angles_from_sides(
        [length_lower_leg, dist_ankle_hip, length_upper_leg]
    )[0:2]

    # The hip angle found with the triangle is not the same as the fe_hip2 angle:
    hip_angle_ankle1_ankle2 = np.sign(ankle_x) * qas.get_angle_between_points(
        [np.array([0, 0]), hip, ankle2]
    )
    fe_hip2 = angle_hip + hip_angle_ankle1_ankle2
    fe_knee2 = knee_zero_angle - angle_knee2

    pose = [0.0, default_hip_aa, 0.0, 0.0, 0.0, default_hip_aa, fe_hip2, fe_knee2]

    if plot:
        current_pose = Pose(pose)
        current_pose.make_plot()

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
    pose_list = [
        ground_pose_flexion,
        default_hip_aa,
        -ground_pose_flexion,
        0,
        -ground_pose_flexion,
        default_hip_aa,
        ground_pose_flexion,
        0,
    ]

    pose = Pose(pose_list)

    # calculate lifted pose if ankle_y > 0:
    if ankle_y > 0:
        pos_ankle = np.array([ankle_x, ankle_y])
        pose = calculate_lifted_pose(pos_ankle, pose)

        # reduce dorsi flexion and straighten leg if fe_ankle2 > max_flexion:
        if pose.fe_ankle2 > max_ankle_flexion:

            # reduce dorsi flexion:
            pose = reduce_dorsi_flexion(max_ankle_flexion, pose)

            # straighten leg:
            pose = straighten_leg(pose)

    if timer:
        end = time.time()
        print("Calculation time = ", end - start, " seconds")

    if plot:
        pose.make_plot()

    pose = pose.get_pose()
    return [np.deg2rad(angle) for angle in pose]

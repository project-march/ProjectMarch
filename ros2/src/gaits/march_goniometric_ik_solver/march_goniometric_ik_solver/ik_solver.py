import numpy as np
import matplotlib.pyplot as plt
import time

import march_goniometric_ik_solver.quadrilateral_angle_solver as qas
import march_goniometric_ik_solver.triangle_angle_solver as tas

from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
    get_limits_robot_from_urdf_for_inverse_kinematics,
)

# Get leg lengths form urdf:
(
    LENGTH_UPPER_LEG,
    LENGTH_LOWER_LEG,
) = get_lengths_robot_from_urdf_for_inverse_kinematics()[0:2]
LENGTH_LEG = LENGTH_UPPER_LEG + LENGTH_LOWER_LEG

# Get ankle limit from urdf:
limits = get_limits_robot_from_urdf_for_inverse_kinematics("right_ankle")
MAX_ANKLE_FLEXION = limits.upper

# Constants:
LENGTH_FOOT = 0.10  # m
ANKLE_ZERO_ANGLE = np.pi / 2  # rad
KNEE_ZERO_ANGLE = np.pi  # rad

HIP_AA = 0.03  # rad


class Pose:
    def __init__(self, pose: list):
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
        self.rot_foot1 = 0

    def calculate_joint_positions(self, joint: str = "all"):
        """
        Calculates the joint positions for a given pose.
        """

        # toe1 is defined at [LENGTH_FOOT, 0]:
        pos_toe1 = np.array([LENGTH_FOOT, 0])

        # ankle1 = toe1 + translation_by_foot:
        pos_ankle1 = np.array(
            [
                pos_toe1[0] - np.cos(self.rot_foot1) * LENGTH_FOOT,
                pos_toe1[1] + np.sin(self.rot_foot1) * LENGTH_FOOT,
            ]
        )

        # knee1 = ankle1 + translation_by_lower_leg:
        pos_knee1 = np.array(
            [
                pos_ankle1[0]
                + np.sin(self.fe_ankle1 + self.rot_foot1) * LENGTH_LOWER_LEG,
                pos_ankle1[1]
                + np.cos(self.fe_ankle1 + self.rot_foot1) * LENGTH_LOWER_LEG,
            ]
        )

        # hip1 = knee1 + translation_by_upper_leg:
        angle_before_knee1 = self.fe_ankle1 + self.rot_foot1
        pos_hip = np.array(
            [
                pos_knee1[0]
                + np.sin(angle_before_knee1 - self.fe_knee1) * LENGTH_UPPER_LEG,
                pos_knee1[1]
                + np.cos(angle_before_knee1 - self.fe_knee1) * LENGTH_UPPER_LEG,
            ]
        )

        # knee2 = hip + translation_by_upper_leg:
        pos_knee2 = np.array(
            [
                pos_hip[0] + np.sin(self.fe_hip2) * LENGTH_UPPER_LEG,
                pos_hip[1] - np.cos(self.fe_hip2) * LENGTH_UPPER_LEG,
            ]
        )

        # ankle2 = knee2 + translation_by_lower_leg:
        pos_ankle2 = np.array(
            [
                pos_knee2[0] + np.sin(self.fe_hip2 - self.fe_knee2) * LENGTH_LOWER_LEG,
                pos_knee2[1] - np.cos(self.fe_hip2 - self.fe_knee2) * LENGTH_LOWER_LEG,
            ]
        )

        # toe2 = ankle2 + translation_by_foot:
        angle_before_ankle2 = self.fe_hip2 - self.fe_knee2
        angle_ankle2 = ANKLE_ZERO_ANGLE + self.fe_ankle2
        pos_toe2 = np.array(
            [
                pos_ankle2[0]
                + np.sin(angle_before_ankle2 + angle_ankle2) * LENGTH_FOOT,
                pos_ankle2[1]
                - np.cos(angle_before_ankle2 + angle_ankle2) * LENGTH_FOOT,
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

    def get_ankle_distance(self):
        pos_ankle1 = self.calculate_joint_positions("pos_ankle1")
        pos_ankle2 = self.calculate_joint_positions("pos_ankle2")
        return np.linalg.norm(pos_ankle1 - pos_ankle2)

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

    def calculate_lifted_pose(self, pos_ankle2: np.array):
        """
        Calculate the pose after lifting the foot to the desired ankle postion.
        """

        # calculate angles using triangle between hip, knee2 and ankle2 and side distances:
        pos_hip = self.calculate_joint_positions("pos_hip")
        dist_hip_ankle = np.linalg.norm(pos_hip - pos_ankle2)
        sides = [LENGTH_LOWER_LEG, dist_hip_ankle, LENGTH_UPPER_LEG]
        angle_hip, angle_knee2, angle_ankle2 = tas.get_angles_from_sides(sides)

        # define new fe_hip2:
        point_below_hip = np.array([pos_ankle2[0] / 2, 0])
        hip_angle_vertical_ankle2 = qas.get_angle_between_points(
            [point_below_hip, pos_hip, pos_ankle2]
        )
        self.fe_hip2 = hip_angle_vertical_ankle2 + angle_hip

        # define new fe_knee2:
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2

        # define new fe_ankle2:
        toe2 = pos_ankle2 + np.array([LENGTH_FOOT, 0])
        ankle2_angle_toe2_hip = qas.get_angle_between_points(
            [toe2, pos_ankle2, pos_hip]
        )
        self.fe_ankle2 = ANKLE_ZERO_ANGLE - (ankle2_angle_toe2_hip - angle_ankle2)

    def reduce_swing_dorsi_flexion(self, max_flexion: float):
        """
        Calculate the pose after reducing the dorsiflexion using quadrilateral solver
        with quadrilateral between ankle2, knee2, hip, knee1
        """

        # get current state:
        (
            pos_ankle1,
            pos_knee1,
            pos_hip,
            pos_knee2,
            pos_ankle2,
        ) = self.calculate_joint_positions()[1:-1]

        # determine angle_ankle2 of quadrilateral:
        reduction = self.fe_ankle2 - max_flexion
        angle_ankle2 = (
            qas.get_angle_between_points([pos_knee1, pos_ankle2, pos_knee2]) - reduction
        )

        # determine other angles using angle_ankle2 and sides:
        dist_knee1_ankle2 = np.linalg.norm(pos_knee1 - pos_ankle2)
        sides = [
            LENGTH_UPPER_LEG,
            dist_knee1_ankle2,
            LENGTH_LOWER_LEG,
            LENGTH_UPPER_LEG,
        ]
        angle_knee1, angle_ankle2, angle_knee2, angle_hip = qas.solve_quadritlateral(
            sides, angle_ankle2
        )

        # define new fe_knee1:
        knee1_angle_ankle1_ankle2 = qas.get_angle_between_points(
            [pos_ankle1, pos_knee1, pos_ankle2]
        )
        self.fe_knee1 = angle_knee1 + knee1_angle_ankle1_ankle2 - KNEE_ZERO_ANGLE

        # get new hip location and determine point below it:
        pos_hip = self.calculate_joint_positions("pos_hip")
        point_below_hip = np.array([pos_hip[0], 0])

        # define new fe_hip1:
        if pos_hip[0] < pos_knee1[0]:
            self.fe_hip1 = qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )
        else:
            self.fe_hip1 = -qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )

        # define new fe_hip2, fe_knee2 and fe_ankle2:
        self.fe_hip2 = angle_hip + self.fe_hip1
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2
        self.fe_ankle2 -= reduction

    def straighten_leg(self):
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
        ) = self.calculate_joint_positions()

        # determine sides of triangle and calculate angles:
        dist_ankle1_knee2 = np.linalg.norm(pos_ankle1 - pos_knee2)
        sides = [LENGTH_UPPER_LEG, LENGTH_LEG, dist_ankle1_knee2]
        angle_ankle1, angle_knee2, angle_hip = tas.get_angles_from_sides(sides)

        # define new fe_ankle1 and fe_knee1:
        ankle1_angle_toe1_knee2 = qas.get_angle_between_points(
            [pos_knee2, pos_ankle1, pos_toe1]
        )
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - (angle_ankle1 + ankle1_angle_toe1_knee2)
        self.fe_knee1 = 0

        # get new knee1 and hip location and determine point below it:
        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        point_below_hip = np.array([pos_hip[0], 0])

        # define new fe_hip1:
        if pos_hip[0] < pos_knee1[0]:
            self.fe_hip1 = qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )
        else:
            self.fe_hip1 = -qas.get_angle_between_points(
                [pos_knee1, pos_hip, point_below_hip]
            )

        # define new fe_hip2 and fe_knee2:
        self.fe_hip2 = angle_hip + self.fe_hip1
        knee2_angle_ankle1_ankle2 = qas.get_angle_between_points(
            [pos_ankle1, pos_knee2, pos_ankle2]
        )
        self.fe_knee2 = KNEE_ZERO_ANGLE - (angle_knee2 + knee2_angle_ankle1_ankle2)

    def reduce_stance_dorsi_flexion(self):
        # Save current angle at toe1 between ankle1 and hip:
        pos_ankle1 = self.calculate_joint_positions("pos_ankle1")
        pos_toe1 = self.calculate_joint_positions("pos_toe1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        toe1_angle_ankle1_hip = qas.get_angle_between_points(
            [pos_ankle1, pos_toe1, pos_hip]
        )

        # Reduce dorsi flexion of stance leg:
        dis_toe1_hip = np.linalg.norm(pos_toe1 - pos_hip)
        lengths = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, LENGTH_FOOT, dis_toe1_hip]
        angle_knee1, angle_ankle1, angle_toe1, angle_hip = qas.solve_quadritlateral(
            lengths=lengths, angle_b=ANKLE_ZERO_ANGLE - MAX_ANKLE_FLEXION, convex=False
        )

        # Update pose:
        self.rot_foot1 = toe1_angle_ankle1_hip - angle_toe1
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - angle_ankle1
        self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee1


def calculate_ground_pose_flexion(ankle_x: float):
    """
    Calculates and returns the flexion of the ankles and the hips when the
    ankle is moved to a certain x position, using pythagoras theorem.
    """

    return np.arcsin((ankle_x / 2) / LENGTH_LEG)


def solve_mid_position(
    ankle_x: float, ankle_y: float, subgait_id: str, plot: bool = False
):
    """
    Solve inverse kinematics for the middle position. Assumes that the
    stance leg is straight. Takes the ankle_x and ankle_y position of the
    desired middle position. Calculates the required hip and knee angles of
    the swing leg by making a triangle between the swing leg ankle, swing leg
    knee and the hip. Returns the calculated pose.
    """

    ankle2 = np.array([ankle_x, ankle_y])
    hip = np.array([0, LENGTH_LEG])
    dist_ankle_hip = np.linalg.norm(ankle2 - hip)

    # Calculate hip and knee2 angle in triangle with ankle2:
    angle_hip, angle_knee2 = tas.get_angles_from_sides(
        [LENGTH_LOWER_LEG, dist_ankle_hip, LENGTH_UPPER_LEG]
    )[0:2]

    # The hip angle found with the triangle is not the same as the fe_hip2 angle:
    hip_angle_ankle1_ankle2 = np.sign(ankle_x) * qas.get_angle_between_points(
        [np.array([0, 0]), hip, ankle2]
    )
    fe_hip2 = angle_hip + hip_angle_ankle1_ankle2
    fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2

    pose = [0.0, HIP_AA, 0.0, 0.0, 0.0, HIP_AA, fe_hip2, fe_knee2]

    if plot:
        current_pose = Pose(pose)
        current_pose.make_plot()

    if subgait_id == "left_swing":
        half1 = pose[: len(pose) // 2]
        half2 = pose[len(pose) // 2 :]
        pose = half2 + half1

    return list(pose)


def solve_end_position(
    ankle_x: float,
    ankle_y: float,
    subgait_id: str,
    max_ankle_flexion: float = MAX_ANKLE_FLEXION,
    plot: bool = False,
    timer: bool = False,
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
        HIP_AA,
        -ground_pose_flexion,
        0,
        -ground_pose_flexion,
        HIP_AA,
        ground_pose_flexion,
        0,
    ]

    pose = Pose(pose_list)

    # calculate lifted pose if ankle_y > 0:
    if ankle_y > 0:
        pos_ankle = np.array([ankle_x, ankle_y])
        pose.calculate_lifted_pose(pos_ankle)

        # reduce dorsi flexion of swing leg and straighten stance leg
        # if fe_ankle2 > max_flexion:
        if pose.fe_ankle2 > max_ankle_flexion:

            # reduce dorsi flexion of swing leg:
            pose.reduce_swing_dorsi_flexion(max_ankle_flexion)

            # straighten stance leg:
            pose.straighten_leg()

    # reduce dorsi flexion of stance leg if fe_ankle1 > MAX_ANKLE_FLEXION:
    if pose.fe_ankle1 > MAX_ANKLE_FLEXION:
        pose.reduce_stance_dorsi_flexion()

    if timer:
        end = time.time()
        print("Calculation time = ", end - start, " seconds")

    if plot:
        pose.make_plot()

    pose_list = pose.get_pose()

    if subgait_id == "left_swing":
        half1 = pose_list[: len(pose_list) // 2]
        half2 = pose_list[len(pose_list) // 2 :]
        pose_list = half2 + half1

    return list(pose_list)

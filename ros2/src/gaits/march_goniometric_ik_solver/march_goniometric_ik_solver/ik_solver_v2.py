import numpy as np
from typing import List
import matplotlib.pyplot as plt

import march_goniometric_ik_solver.triangle_angle_solver as tas
import march_goniometric_ik_solver.quadrilateral_angle_solver as qas

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
SOFT_LIMIT_BUFFER = np.deg2rad(5)
MAX_ANKLE_FLEXION = limits.upper - SOFT_LIMIT_BUFFER

# Constants:
LENGTH_FOOT = 0.10  # m

ANKLE_ZERO_ANGLE = np.pi / 2  # rad
KNEE_ZERO_ANGLE = np.pi  # rad
HIP_ZERO_ANGLE = np.pi  # rad

HIP_AA = 0.03  # rad

NUMBER_OF_JOINTS = 8

DEFAULT_KNEE_BEND = np.deg2rad(8)


class Pose:
    """
    Used to solve inverse kinematics for a desired end_postion or mid_position of the foot.
    The class contains the joint_angles and the foot_rotation of the rear foot (in case of a toe-off).
    Solving can be done for the left or right foot, therefore this class uses the definition of 1 or 2
    for the joints, where 1 is the rear leg and 2 the front leg.
    Positive defined are: ankle dorsi-flexion, hip abduction, hip flexion, knee flexion.
    More documentation about the inverse kinematic method can be found on confluence:
    https://confluence.projectmarch.nl/x/vo6AFw
    """

    def __init__(self, pose: List[float] = [0.0] * NUMBER_OF_JOINTS):
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
        self.ankle_x = 0
        self.ankle_y = 0

    def reset_to_zero_pose(self):
        self.__init__()

    @property
    def pose_right(self) -> List[float]:
        """
        Returns the pose as list with the right leg as front leg (leg2):
        """
        return [
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
        ]

    @property
    def pose_left(self) -> List[float]:
        """
        Returns the pose as list with the left leg as front leg (leg2):
        """
        return [
            self.fe_ankle2,
            self.aa_hip2,
            self.fe_hip2,
            self.fe_knee2,
            self.fe_ankle1,
            self.aa_hip1,
            self.fe_hip1,
            self.fe_knee1,
        ]

    def calculate_joint_positions(self, joint: str = "all") -> tuple:
        """
        Calculates the joint positions for a given pose as a chain from rear toes (toes1) to front toes (toes2).
        Can return all joint positions or a specific one, by choosing from:
        pos_toes1, pos_ankle1, pos_knee1, pos_hip, pos_knee2, pos_ankle2, pos_toes2

        If a positive angle represents a anti-clockwise rotation, the angle variable is positive in the rot function.
        If a positive angle represents a clockwise rotation, the angle variable is negative in the rot function.
        The vectors (all np.array's) do always describe the translation when no rotation is applied.
        The rot_total matrix expands every step, since every joint location depends on all previous joint angles in the chain.
        """
        # create rotation matrix that we expand after every rotation:
        rot_total = rot(0)

        # toes1 is defined at [LENGTH_FOOT, 0]:
        pos_toes1 = np.array([LENGTH_FOOT, 0])

        # ankle1 = toes1 + translation_by_foot1:
        rot_foot = rot(-self.rot_foot1)
        rot_total = rot_total @ rot_foot
        pos_ankle1 = pos_toes1 + rot_total @ np.array([-LENGTH_FOOT, 0])

        # knee1 = ankle1 + translation_by_lower_leg1:
        rot_ankle1 = rot(-self.fe_ankle1)
        rot_total = rot_total @ rot_ankle1
        pos_knee1 = pos_ankle1 + rot_total @ np.array([0, LENGTH_LOWER_LEG])

        # hip = knee1 + translation_by_upper_leg1:
        rot_knee1 = rot(self.fe_knee1)
        rot_total = rot_total @ rot_knee1
        pos_hip = pos_knee1 + rot_total @ np.array([0, LENGTH_UPPER_LEG])

        # knee2 = hip + translation_by_upper_leg2:
        rot_hip = rot(self.fe_hip2 - self.fe_hip1)
        rot_total = rot_total @ rot_hip
        pos_knee2 = pos_hip + rot_total @ np.array([0, -LENGTH_UPPER_LEG])

        # ankle2 = knee2 + translation_by_lower_leg2:
        rot_knee2 = rot(-self.fe_knee2)
        rot_total = rot_total @ rot_knee2
        pos_ankle2 = pos_knee2 + rot_total @ np.array([0, -LENGTH_LOWER_LEG])

        # toe2 = ankle2 + translation_by_foot2:
        rot_ankle2 = rot(self.fe_ankle2)
        rot_total = rot_total @ rot_ankle2
        pos_toes2 = pos_ankle2 + rot_total @ np.array([LENGTH_FOOT, 0])

        # return all positions, or one specific if asked:
        if joint == "all":
            return (
                pos_toes1,
                pos_ankle1,
                pos_knee1,
                pos_hip,
                pos_knee2,
                pos_ankle2,
                pos_toes2,
            )
        else:
            return locals()[joint]

    @property
    def hip_x(self) -> float:
        return self.ankle_x * self.hip_x_fraction

    @property
    def max_leg_length(self) -> float:
        pose = Pose()
        pose.fe_knee1 = self.default_knee_bend
        pos_hip = pose.calculate_joint_positions("pos_hip")
        pos_ankle1 = pose.calculate_joint_positions("pos_ankle1")
        return np.linalg.norm(pos_hip - pos_ankle1)

    def bended_angles(self, leg_length: float):
        if leg_length < LENGTH_LEG:
            sides = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, leg_length]
            bend_ankle, bend_hip, bend_knee = tas.get_angles_from_sides(sides)
            return bend_ankle, bend_hip, bend_knee
        else:
            return 0.0, 0.0, np.pi

    def solve_rear_leg(self, pos_hip, pos_ankle1):
        dist_ankle1_hip = np.linalg.norm(pos_hip - pos_ankle1)
        bend_ankle1, bend_hip1, bend_knee1 = self.bended_angles(dist_ankle1_hip)
        angle_rear = np.arccos(pos_hip[1] / dist_ankle1_hip)

        self.fe_ankle1 = angle_rear + bend_ankle1
        self.fe_knee1 = KNEE_ZERO_ANGLE - bend_knee1
        self.fe_hip1 = -angle_rear + bend_hip1

    def solve_front_leg(self, pos_hip, pos_ankle2):
        dist_ankle2_hip = np.linalg.norm(pos_hip - pos_ankle2)
        bend_ankle2, bend_hip2, bend_knee2 = self.bended_angles(dist_ankle2_hip)
        angle_front = np.arcsin((self.ankle_x - self.hip_x) / dist_ankle2_hip)

        self.fe_hip2 = angle_front + bend_hip2
        self.fe_knee2 = KNEE_ZERO_ANGLE - bend_knee2
        self.fe_ankle2 = -angle_front + bend_ankle2

    def reduce_stance_dorsi_flexion(self):
        # Save current angle at toes1 between ankle1 and hip:
        pos_ankle1 = self.calculate_joint_positions("pos_ankle1")
        pos_toes1 = self.calculate_joint_positions("pos_toes1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        toes1_angle_ankle1_hip = qas.get_angle_between_points(
            [pos_ankle1, pos_toes1, pos_hip]
        )

        # Reduce dorsi flexion of stance leg:
        dis_toes1_hip = np.linalg.norm(pos_toes1 - pos_hip)
        lengths = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, LENGTH_FOOT, dis_toes1_hip]
        angle_knee1, angle_ankle1, angle_toes1, angle_hip = qas.solve_quadritlateral(
            lengths=lengths, angle_b=ANKLE_ZERO_ANGLE - MAX_ANKLE_FLEXION, convex=False
        )

        # Update pose:
        self.rot_foot1 = toes1_angle_ankle1_hip - angle_toes1
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - angle_ankle1
        self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee1

        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        point_below_hip = np.array([pos_hip[0], 0])
        self.fe_hip1 = np.sign(
            pos_knee1[0] - pos_hip[0]
        ) * qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

    def solve_all(
        self,
        ankle_x,
        ankle_y,
        hip_x_fraction,
        default_knee_bend,
        reduce_df_rear,
    ):
        # set parameters:
        self.ankle_x = ankle_x
        self.ankle_y = ankle_y
        self.hip_x_fraction = hip_x_fraction
        self.default_knee_bend = default_knee_bend

        # determine hip y-location:
        if ankle_y > 0:
            if hip_x_fraction >= 0.5:
                hip_y = np.sqrt(self.max_leg_length ** 2 - (self.hip_x) ** 2)
            else:
                hip_y = np.sqrt(self.max_leg_length ** 2 - (ankle_x - self.hip_x) ** 2)
        else:
            hip_y = min(
                ankle_y
                + np.sqrt(self.max_leg_length ** 2 - (ankle_x - self.hip_x) ** 2),
                np.sqrt(self.max_leg_length ** 2 - (self.hip_x) ** 2),
            )

        # define locations:
        pos_hip = np.array([self.hip_x, hip_y])
        pos_ankle1 = np.array([0, 0])
        pos_ankle2 = np.array([ankle_x, ankle_y])

        # rear leg:
        self.solve_rear_leg(pos_hip, pos_ankle1)

        # front leg:
        self.solve_front_leg(pos_hip, pos_ankle2)

        # reduce stance dorsi flexion:
        if reduce_df_rear and self.fe_ankle1 > MAX_ANKLE_FLEXION:
            self.reduce_stance_dorsi_flexion()


# Static methods:
def rot(t: float) -> np.array:
    """
    Returns the 2D rotation matrix R to rotate a vector with rotation t (in rad), so that:
    ⎡x'⎤ ⎽ ⎡cos(t) -sin(t)⎤⎡x⎤
    ⎣y'⎦ ⎺ ⎣sin(t)  cos(t)⎦⎣y⎦
    A positive value of t results in a anti-clockwise rotation around the origin.
    """
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])


def make_plot(pose: Pose):
    """
    Makes a plot of the exo by first calculating the joint positions and then plotting them.
    This method is only used for debugging reasons.
    """

    positions = pose.calculate_joint_positions()
    positions_x = [pos[0] for pos in positions]
    positions_y = [pos[1] for pos in positions]

    plt.figure(1)
    plt.plot(positions_x[0], positions_y[0], "o")
    plt.plot(positions_x, positions_y, ".-")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()

import numpy as np
from typing import List
import matplotlib.pyplot as plt

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
SOFT_LIMIT_BUFFER = np.deg2rad(2)  # deg
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
        self.aa_hip1 = self.aa_hip2 = HIP_AA

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

    def get_ankle_distance(self) -> float:
        """
        Method to get the distance between the two ankles.
        """
        pos_ankle1 = self.calculate_joint_positions("pos_ankle1")
        pos_ankle2 = self.calculate_joint_positions("pos_ankle2")
        return np.linalg.norm(pos_ankle1 - pos_ankle2)

    def calculate_ground_pose(self, ankle_x: float):
        """
        Calculate the ground pose to reach the x-location of the desired ankle postion,
        while keeping the knees bended with the default knee bend value.
        First it calculates the hip-ankle-length with a bendend knee, next it set the joint values.
        """

        # Determine length_hip_ankle:
        self.fe_knee1 = self.fe_knee2 = DEFAULT_KNEE_BEND
        pos_ankle1 = self.calculate_joint_positions("pos_ankle1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        length_ankle_hip = np.linalg.norm(pos_ankle1 - pos_hip)

        angle_ankle1, angle_knee1, angkle_hip = tas.get_angles_from_sides(
            [LENGTH_UPPER_LEG, length_ankle_hip, LENGTH_LOWER_LEG]
        )

        # Calculate theta as defined in the README:
        theta = np.arcsin(ankle_x / (2 * length_ankle_hip))

        # Define new value of ankle:
        self.fe_ankle1 = theta + angle_ankle1

        # Determine new positions of knee1 and hip and define other joint values:
        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        pos_below_hip = np.array([pos_hip[0], 0])

        self.fe_hip1 = np.sign(
            pos_knee1[0] - pos_hip[0]
        ) * qas.get_angle_between_points([pos_knee1, pos_hip, pos_below_hip])
        self.fe_hip2 = theta + angkle_hip
        self.fe_ankle2 = -theta + angle_ankle1

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
        self.fe_hip1 = np.sign(
            pos_knee1[0] - pos_hip[0]
        ) * qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

        # define new fe_hip2, fe_knee2 and fe_ankle2:
        self.fe_hip2 = angle_hip + self.fe_hip1
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2
        self.fe_ankle2 -= reduction

    def straighten_leg(self):
        """
        Straighten stance leg until the default knee bend by making a quadtrilateral between
        ankle1, knee1, hip, knee2 and calculating new angles.
        """

        # get current state:
        (
            pos_toes1,
            pos_ankle1,
            pos_knee1,
            pos_hip,
            pos_knee2,
            pos_ankle2,
            pos_toes2,
        ) = self.calculate_joint_positions()

        # determine sides of quadrilateral and calculate angles:
        dist_ankle1_knee2 = np.linalg.norm(pos_ankle1 - pos_knee2)
        sides = [
            LENGTH_UPPER_LEG,
            LENGTH_UPPER_LEG,
            LENGTH_LOWER_LEG,
            dist_ankle1_knee2,
        ]
        angle_knee1 = KNEE_ZERO_ANGLE + DEFAULT_KNEE_BEND
        angle_hip, angle_knee1, angle_ankle1, angle_knee2 = qas.solve_quadritlateral(
            sides, angle_knee1, convex=True
        )  # note: this is actually a concave quadrilateral, but qas works strange for angle_b > 180 deg.
        # this should be improved, see issue: https://gitlab.com/project-march/march/-/issues/1362

        # define new fe_ankle1 and fe_knee1:
        ankle1_angle_toes1_knee2 = qas.get_angle_between_points(
            [pos_knee2, pos_ankle1, pos_toes1]
        )
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - (angle_ankle1 + ankle1_angle_toes1_knee2)
        self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee1

        # get new knee1 and hip location and determine point below it:
        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        pos_hip = self.calculate_joint_positions("pos_hip")
        point_below_hip = np.array([pos_hip[0], 0])

        # define new fe_hip1:
        self.fe_hip1 = np.sign(
            pos_knee1[0] - pos_hip[0]
        ) * qas.get_angle_between_points([pos_knee1, pos_hip, point_below_hip])

        # define new fe_hip2 and fe_knee2:
        self.fe_hip2 = angle_hip + self.fe_hip1
        knee2_angle_ankle1_ankle2 = qas.get_angle_between_points(
            [pos_ankle1, pos_knee2, pos_ankle2]
        )
        self.fe_knee2 = KNEE_ZERO_ANGLE - (angle_knee2 + knee2_angle_ankle1_ankle2)

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

    def solve_mid_position(
        self,
        ankle_x: float,
        ankle_y: float,
        midpoint_fraction: float,
        midpoint_height: float,
        subgait_id: str,
    ) -> List[float]:
        """
        Solve inverse kinematics for the middle position. Assumes that the
        stance has a knee flexion of DEFAULT_KNEE_BEND. Takes the ankle_x and
        ankle_y position of the desired position and the midpoint_fraction at
        which a midpoint is desired. First calculates the midpoint position using
        current pose and fraction. Next, calculates the required hip and knee angles of
        the swing leg by making a triangle between the swing leg ankle, swing leg
        knee and the hip. Returns the calculated pose.
        """

        # Get swing distance in current pose and calculate ankle2 midpoint location:
        swing_distance = self.get_ankle_distance()
        midpoint_x = midpoint_fraction * (swing_distance + ankle_x) - swing_distance
        midpoint_y = ankle_y + midpoint_height
        pos_ankle2 = np.array([midpoint_x, midpoint_y])

        # Reset pose to zero_pose and calculate distance between hip and ankle2 midpoint location:
        self.reset_to_zero_pose()
        self.fe_knee1 = DEFAULT_KNEE_BEND
        pos_hip = self.calculate_joint_positions("pos_hip")
        dist_ankle2_hip = np.linalg.norm(pos_ankle2 - pos_hip)

        # Calculate hip and knee2 angles in triangle with ankle2:
        angle_hip, angle_knee2, angle_ankle2 = tas.get_angles_from_sides(
            [LENGTH_LOWER_LEG, dist_ankle2_hip, LENGTH_UPPER_LEG]
        )

        # fe_hip2 = angle_hip +- hip angle between ankle2 and knee1:
        pos_knee1 = self.calculate_joint_positions("pos_knee1")
        hip_angle_ankle2_knee1 = qas.get_angle_between_points(
            [pos_ankle2, pos_hip, pos_knee1]
        )
        self.fe_hip2 = angle_hip + np.sign(midpoint_x) * hip_angle_ankle2_knee1

        # update fe_knee2:
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2

        # lift toes as much as possible:
        self.fe_ankle2 = MAX_ANKLE_FLEXION
        return self.pose_left if (subgait_id == "left_swing") else self.pose_right

    def solve_end_position(
        self,
        ankle_x: float,
        ankle_y: float,
        subgait_id: str,
        max_ankle_flexion: float = MAX_ANKLE_FLEXION,
    ) -> List[float]:
        """
        Solve inverse kinematics for a desired ankle location, assuming flat feet.
        Expects at least the ankle x-position and returns the calculated pose.
        """

        # make sure to start in zero_pose:
        self.reset_to_zero_pose()

        # calculate ground pose:
        self.calculate_ground_pose(ankle_x)

        # calculate lifted pose if ankle_y > 0:
        if ankle_y > 0:
            pos_ankle = np.array([ankle_x, ankle_y])
            self.calculate_lifted_pose(pos_ankle)

            # reduce dorsi flexion of swing leg and straighten stance leg
            # if fe_ankle2 > max_flexion:
            if self.fe_ankle2 > max_ankle_flexion:

                # reduce dorsi flexion of swing leg:
                self.reduce_swing_dorsi_flexion(max_ankle_flexion)

                # straighten stance leg:
                self.straighten_leg()

        # reduce dorsi flexion of stance leg if fe_ankle1 > MAX_ANKLE_FLEXION:
        if self.fe_ankle1 > MAX_ANKLE_FLEXION:
            self.reduce_stance_dorsi_flexion()

        # return pose as list:
        return self.pose_left if (subgait_id == "left_swing") else self.pose_right


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
    plt.plot(positions_x, positions_y)
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()

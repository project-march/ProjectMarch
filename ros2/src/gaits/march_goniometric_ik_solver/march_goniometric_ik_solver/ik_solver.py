"""Author: Jelmer de Wolde, MVII."""

import numpy as np
from typing import List, Tuple, Union
import matplotlib.pyplot as plt

import march_goniometric_ik_solver.triangle_angle_solver as tas
import march_goniometric_ik_solver.quadrilateral_angle_solver as qas

from march_utility.utilities.utility_functions import (
    get_lengths_robot_from_urdf_for_inverse_kinematics,
    get_limits_robot_from_urdf_for_inverse_kinematics,
    get_joint_names_from_urdf,
)

# Get lengths from urdf:
(
    LENGTH_UPPER_LEG,
    LENGTH_LOWER_LEG,
    LENGTH_HIP_AA,
    LENGTH_HIP_BASE,
) = get_lengths_robot_from_urdf_for_inverse_kinematics(
    length_names=["upper_leg", "lower_leg", "hip_aa_front", "hip_base"]
)
LENGTH_LEG = LENGTH_UPPER_LEG + LENGTH_LOWER_LEG
LENGTH_HIP = 2 * LENGTH_HIP_AA + LENGTH_HIP_BASE

# List the joints we have:
JOINT_NAMES = get_joint_names_from_urdf()

# Create a dictionary of joint limits:
JOINT_LIMITS = {}

for name in JOINT_NAMES:
    JOINT_LIMITS[name] = get_limits_robot_from_urdf_for_inverse_kinematics(name)

# Create a constant for frequently used limits:
ANKLE_BUFFER = np.deg2rad(1)
MAX_ANKLE_FLEXION = get_limits_robot_from_urdf_for_inverse_kinematics("left_ankle").upper - ANKLE_BUFFER

# Constants:
LENGTH_FOOT = 0.10  # m

ANKLE_ZERO_ANGLE = np.pi / 2  # rad
KNEE_ZERO_ANGLE = np.pi  # rad
HIP_ZERO_ANGLE = np.pi  # rad

DEFAULT_HIP_X_FRACTION = 0.5
DEFAULT_KNEE_BEND = np.deg2rad(8)


class Pose:
    """Pose class used for inverse kinematic calculations.

    Used to solve inverse kinematics for a desired end_postion or mid_position of the foot.
    The class contains the joint_angles and the foot_rotation of the rear foot (in case of a toe-off)
    Solving can be done for the left or right foot, therefore this class uses the definition of 1 or 2
    for the joints, where 1 is the rear leg and 2 the front leg.
    Positive defined are: ankle dorsi-flexion, hip abduction, hip flexion, knee flexion.

    Args:
        pose (List[float]): List of the joint angles for the pose.

    Attributes:
        fe_ankle1 (float): dorsi-flexion (flexion) or plantar-flexion (extension) of the ankle of the hip on the stance leg.
        aa_hip1 (float): adduction or abduction of hip on the stance leg.
        fe_hip1 (float): flexion or extension of the hip on the stance leg.
        fe_knee1 (float): flexion or extension of the knee on the stance leg.
        fe_ankle2 (float): dorsi-flexion (flexion) or plantar-flexion (extension) of the ankle of the hip on the swing leg.
        aa_hip2 (float): adduction or abduction of hip on the swing leg.
        fe_hip2 (float): flexion or extension of the hip on the swing leg.
        fe_knee2 (float): flexion or extension of the knee on the swing leg.
        rot_foot1 (float): angle between flat ground and the foot on the stance leg.
    """

    def __init__(self, all_joint_names: List[str] = JOINT_NAMES, pose: List[float] = None) -> None:
        self.all_joint_names = all_joint_names
        if pose is None:
            angle_ankle, angle_hip, angle_knee = self.leg_length_angles(self.max_leg_length)
            self.fe_ankle1 = self.fe_ankle2 = angle_ankle
            self.fe_hip1 = self.fe_hip2 = angle_hip
            self.fe_knee1 = self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee
            self.aa_hip1 = self.aa_hip2 = 0
        else:
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

    def reset_to_zero_pose(self) -> None:
        """Reset the post to the default zero pose."""
        self.__init__(self.all_joint_names)

    @property
    def pose_right(self) -> List[float]:
        """Returns the pose as list with the right leg as the swing leg (leg2)."""
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
        """Returns the pose as list with the left leg as swing leg (leg2)."""
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

    def calculate_joint_positions(self, joint: str = "all") -> Union[Tuple[float], float]:
        """Calculates the joint positions for a given pose as a chain from rear toes (toes1) to front toes (toes2).

        If a positive angle represents an anti-clockwise rotation, the angle variable is positive in the rot function.
        If a positive angle represents a clockwise rotation, the angle variable is negative in the rot function.
        The vectors (all np.array's) do always describe the translation when no rotation is applied.
        The rot_total matrix expands every step, since every joint location depends on all previous joint angles in the
        chain.

        Args:
            joint (str): specific joint of which the joint position should be returned.
                default argument is 'all', which returns a list joint positions for all joints.
                possible arguments are: pos_toes1, pos_ankle1, pos_knee1, pos_hip, pos_knee2, pos_ankle2, pos_toes2.

        Returns:
            tuple/float: returns a tuple if all joint positions are asked, otherwise a float of the single joint position.
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
    def pos_toes1(self) -> np.array:
        """np.Array[x,y]. Calculates position of the stance leg toes."""
        return self.calculate_joint_positions("pos_toes1")

    @property
    def pos_ankle1(self) -> np.array:
        """np.Array[x,y]. Calculates position of the stance leg ankle."""
        return self.calculate_joint_positions("pos_ankle1")

    @property
    def pos_knee1(self) -> np.array:
        """np.Array[x,y]. Calculates position of the stance leg knee."""
        return self.calculate_joint_positions("pos_knee1")

    @property
    def pos_hip(self) -> np.array:
        """np.Array[x,y]. Calculates position of the hip."""
        return self.calculate_joint_positions("pos_hip")

    @property
    def pos_knee2(self) -> np.array:
        """np.Array[x,y]. Calculates position of the swing leg knee."""
        return self.calculate_joint_positions("pos_knee2")

    @property
    def pos_ankle2(self) -> np.array:
        """np.Array[x,y]. Calculates position of the swing leg ankle."""
        return self.calculate_joint_positions("pos_ankle2")

    @property
    def pos_toes2(self) -> np.array:
        """np.Array[x,y]. Calculates position of the swing leg toes."""
        return self.calculate_joint_positions("pos_toes2")

    @property
    def point_below_hip(self) -> np.array:
        """np.Array[x,y]. Returns a ground point below the hip."""
        return np.array([self.pos_hip[0], 0])

    @property
    def hip_x(self) -> float:
        """float. Returns hip x position, based on how far forward the hip is placed."""
        return self.ankle_x * self.hip_x_fraction

    @property
    def max_leg_length(self) -> float:
        """Returns the max net leg length (between ankle and hip) for the given knee_bend of the pose object."""
        pos_ankle = np.array([0, 0])
        pos_knee = pos_ankle + np.array([0, LENGTH_LOWER_LEG])

        try:
            knee_angle = self.knee_bend
        except AttributeError:
            knee_angle = DEFAULT_KNEE_BEND

        pos_hip = pos_knee + rot(knee_angle) @ np.array([0, LENGTH_UPPER_LEG])
        return np.linalg.norm(pos_hip - pos_ankle)

    @property
    def ankle_limit_toes_knee_distance(self) -> float:
        """Returns the distance between knee and toes when the ankle is in max dorsi flexion."""
        pose = Pose(self.all_joint_names)
        pose.fe_ankle1 = MAX_ANKLE_FLEXION
        return np.linalg.norm(pose.pos_toes1 - pose.pos_knee1)

    @property
    def ankle_limit_toes_hip_distance(self) -> float:
        """Returns the distance between hip and toes when the ankle is in max dorsi flexion."""
        pose = Pose(self.all_joint_names)
        pose.fe_ankle1 = MAX_ANKLE_FLEXION
        return np.linalg.norm(pose.pos_toes1 - pose.pos_hip)

    def leg_length_angles(self, leg_length: float) -> Tuple[float]:
        """Returns the required angles in the triangle between ankle, hip and knee to meet the leg_length.

        Args:
            leg_length (float): the desired distance between ankle and hip.

        Returns:
            Tuple[float]: returns the required angles in order: ankle, hip knee.
        """
        if leg_length < LENGTH_LEG:
            sides = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, leg_length]
            angle_ankle, angled_hip, angle_knee = tas.get_angles_from_sides(sides)
            return angle_ankle, angled_hip, angle_knee
        else:
            return 0.0, 0.0, np.pi

    def solve_leg(self, pos_hip: np.array, pos_ankle: np.array, leg: str) -> None:
        """Solve the joint angles for a given leg to have hip and ankle at given positions.

        Args:
            pos_hip (np.array): a 2D numpy array containing the position of the hip
            pos_ankle (np.array): a 2D numpy array containing the position of the ankle
            leg (str): defines for which leg it needs to solve, can be 'rear' or 'front'.
        """
        dist_ankle_hip = np.linalg.norm(pos_hip - pos_ankle)
        angle_ankle, angle_hip, angle_knee = self.leg_length_angles(dist_ankle_hip)

        base_angle = np.arcsin(abs(pos_ankle[0] - pos_hip[0]) / dist_ankle_hip)

        if leg == "rear":
            self.fe_ankle1 = base_angle + angle_ankle
            self.fe_hip1 = -base_angle + angle_hip
            self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee

        elif leg == "front":
            self.fe_ankle2 = -base_angle + angle_ankle
            self.fe_hip2 = base_angle + angle_hip
            self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee

        else:
            raise ValueError("Expected leg to be 'rear' or 'front'.")

    def reduce_swing_dorsi_flexion(self) -> None:
        """Calculates the pose after reducing the dorsiflexion for the swing leg.

        Creates a quadrilateral between ankle2, knee2, hip, knee1.
        Reduces the angle at the ankle so that dorsiflexion is within the limit.
        Next it defines the other angles in the quadrilateral with this new ankle angle.
        Finally it updates the pose with the reduced dorsiflexion.
        """
        # Determine required reduction:
        reduction = self.fe_ankle2 - MAX_ANKLE_FLEXION

        # Store current angle of ankle1 between ankle2 and hip:
        angle_ankle1_before = qas.get_angle_between_points([self.pos_ankle2, self.pos_ankle1, self.pos_hip])

        # Define desired angle_ankle2 and determine other angles in quadrilateral:
        angle_ankle2 = qas.get_angle_between_points([self.pos_ankle1, self.pos_ankle2, self.pos_knee2]) - reduction
        dist_ankle1_ankle2 = np.linalg.norm(self.pos_ankle1 - self.pos_ankle2)
        sides = [
            self.max_leg_length,
            dist_ankle1_ankle2,
            LENGTH_LOWER_LEG,
            LENGTH_UPPER_LEG,
        ]
        angle_ankle1, angle_ankle2, angle_knee2, angle_hip = qas.solve_quadritlateral(sides, angle_ankle2)

        # Update the pose:
        self.fe_ankle1 = self.fe_ankle1 - (angle_ankle1 - angle_ankle1_before)
        self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )
        self.fe_hip2 = (
            angle_hip - qas.get_angle_between_points([self.pos_ankle1, self.pos_hip, self.pos_knee1]) + self.fe_hip1
        )
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2
        self.fe_ankle2 -= reduction

    def keep_hip_above_rear_ankle(self) -> None:
        """Calculates the pose required to keep the hip above the rear ankle while reaching the goal location for the toes."""
        # Store desired toes location and reset pose:
        pos_toes2 = np.array([self.ankle_x + LENGTH_FOOT, self.ankle_y])
        self.reset_to_zero_pose()

        # Calculate angles in triangle between hip, toes2 and knee2:
        dist_hip_toes2 = np.linalg.norm(self.pos_hip - pos_toes2)
        dist_toes2_knee2 = self.ankle_limit_toes_knee_distance
        angle_hip, angle_toes2, angle_knee2 = tas.get_angles_from_sides(
            [dist_toes2_knee2, LENGTH_UPPER_LEG, dist_hip_toes2]
        )

        # Calculate outer angles of hip and knee:
        angle_hip_out = qas.get_angle_between_points([self.point_below_hip, self.pos_hip, pos_toes2])
        angle_knee2_out = tas.get_angle_from_sides(LENGTH_FOOT, np.array([LENGTH_LOWER_LEG, dist_toes2_knee2]))

        # Update the pose:
        self.fe_hip2 = angle_hip + angle_hip_out
        self.fe_knee2 = KNEE_ZERO_ANGLE - (angle_knee2 - angle_knee2_out)
        self.fe_ankle2 = MAX_ANKLE_FLEXION

    def reduce_stance_dorsi_flexion(self) -> None:
        """Calculates the pose after reducing the dorsiflexion for the stance leg.

        Creates a quadrilateral between toes1, ankle1, knee1, and hip.
        Reduces the angle at the ankle so that dorsiflexion is within the limit.
        Next it defines the other angles in the quadrilateral with this new ankle angle.
        Finally it updates the pose with the reduced dorsiflexion.
        """
        # Save current angle at toes1 between ankle1 and hip:
        toes1_angle_ankle1_hip = qas.get_angle_between_points([self.pos_ankle1, self.pos_toes1, self.pos_hip])

        # Reduce dorsi flexion of stance leg:
        dis_toes1_hip = np.linalg.norm(self.pos_toes1 - self.pos_hip)
        lengths = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, LENGTH_FOOT, dis_toes1_hip]
        angle_knee1, angle_ankle1, angle_toes1, angle_hip = qas.solve_quadritlateral(
            lengths=lengths, angle_b=ANKLE_ZERO_ANGLE - MAX_ANKLE_FLEXION, convex=False
        )

        # Update pose:
        self.rot_foot1 = toes1_angle_ankle1_hip - angle_toes1
        self.fe_ankle1 = ANKLE_ZERO_ANGLE - angle_ankle1
        self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee1
        self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )

    def perform_side_step(self, y: float, z: float):
        """Calculates the required hip adduction/abduction to step sidewards.

        Updates the pose to add the sidewards step, without changing the forward or upward step already in the pose.
        Requires very complex equations, see the README.MD of this package for more information.

        Args:
            y (float): the distance between the two feet in upward direction.
            z (float): the distance between the two feet in sidewards direction.
        """
        # Get y position of hip and toes:
        y_hip = self.pos_hip[1]
        y_toes1 = self.pos_toes1[1]
        y_toes2 = self.pos_toes2[1]

        # Determine lengths:
        length_leg_short, length_leg_long = sorted([y_hip - y_toes1, y_hip - y_toes2])
        length_short, length_long = (
            np.sqrt(length_leg_short ** 2 + LENGTH_HIP_AA ** 2),
            np.sqrt(length_leg_long ** 2 + LENGTH_HIP_AA ** 2),
        )

        # Calculate theta_long and theta_short:
        # Note that flake8-expression-complexity check is disabled since this is a very complex calculation.
        theta_long = 2 * np.arctan(  # noqa: ECE001
            (
                -2 * length_long * LENGTH_HIP_BASE
                + 2 * length_long * z
                - np.sqrt(
                    -(length_long ** 4)
                    + 2 * length_long ** 2 * length_short ** 2
                    + 2 * length_long ** 2 * LENGTH_HIP_BASE ** 2
                    - 4 * length_long ** 2 * LENGTH_HIP_BASE * z
                    + 2 * length_long ** 2 * y ** 2
                    + 2 * length_long ** 2 * z ** 2
                    - length_short ** 4
                    + 2 * length_short ** 2 * LENGTH_HIP_BASE ** 2
                    - 4 * length_short ** 2 * LENGTH_HIP_BASE * z
                    + 2 * length_short ** 2 * y ** 2
                    + 2 * length_short ** 2 * z ** 2
                    - LENGTH_HIP_BASE ** 4
                    + 4 * LENGTH_HIP_BASE ** 3 * z
                    - 2 * LENGTH_HIP_BASE ** 2 * y ** 2
                    - 6 * LENGTH_HIP_BASE ** 2 * z ** 2
                    + 4 * LENGTH_HIP_BASE * y ** 2 * z
                    + 4 * LENGTH_HIP_BASE * z ** 3
                    - y ** 4
                    - 2 * y ** 2 * z ** 2
                    - z ** 4
                )
            )
            / (
                length_long ** 2
                + 2 * length_long * y
                - length_short ** 2
                + LENGTH_HIP_BASE ** 2
                - 2 * LENGTH_HIP_BASE * z
                + y ** 2
                + z ** 2
            )
        )

        theta_short = np.arccos((length_long * np.cos(theta_long) - y) / length_short)

        # Determine hip_aa for both hips:
        angle_hip_long = tas.get_angle_from_sides(length_leg_long, np.array([length_long, LENGTH_HIP_AA]))
        angle_hip_short = tas.get_angle_from_sides(length_leg_short, np.array([length_short, LENGTH_HIP_AA]))

        hip_aa_long = theta_long + angle_hip_long - HIP_ZERO_ANGLE / 2
        hip_aa_short = theta_short + angle_hip_short - HIP_ZERO_ANGLE / 2

        # Update pose:
        if y_toes1 > y_toes2:
            self.aa_hip1 = hip_aa_short
            self.aa_hip2 = hip_aa_long
        else:
            self.aa_hip1 = hip_aa_long
            self.aa_hip2 = hip_aa_short

    def solve_mid_position(
        self,
        ankle_x: float,
        ankle_y: float,
        ankle_z: float,
        midpoint_fraction: float,
        midpoint_height: float,
        subgait_id: str,
    ) -> List[float]:
        """Solves inverse kinematics for the middle position.

        Assumes that the stance leg is straight with a knee flexion of DEFAULT_KNEE_BEND.
        First it calculates the desired foot location using end position, midpoint fraction and midpoint height.
        Next it resets to zero pose and calculates the required angles for the swing leg to reach the calculated midpoint.

        Args:
            ankle_x (float): the forward distance for the end position.
            ankle_y (float): the upward distance for the end position.
            ankle_z (float): the sideward distance for the end position.
            midpoint_fraction (float): the fraction of the step at which the mid position should be.
            midpoint_height: the height the mid position should be relative to the end position.
            subgait_id (str): either 'left_swing' or 'right_swing', defines which leg is the swing leg.

        Returns:
            List[float]: a list of all the joint angles to perform the desired mid position.
        """
        # Get swing distance in current pose and calculate ankle2 midpoint location:
        swing_distance = np.linalg.norm(self.pos_ankle1 - self.pos_ankle2)
        midpoint_x = midpoint_fraction * (swing_distance + ankle_x) - swing_distance
        midpoint_y = ankle_y + midpoint_height
        pos_ankle2 = np.array([midpoint_x, midpoint_y])

        # Store start pose:
        start_hip_aa1 = self.aa_hip1
        start_hip_aa2 = self.aa_hip1

        # Reset pose to zero_pose and calculate distance between hip and ankle2 midpoint location:
        self.reset_to_zero_pose()
        dist_ankle2_hip = np.linalg.norm(pos_ankle2 - self.pos_hip)

        # Calculate hip and knee2 angles in triangle with ankle2:
        angle_hip, angle_knee2, angle_ankle2 = tas.get_angles_from_sides(
            [LENGTH_LOWER_LEG, dist_ankle2_hip, LENGTH_UPPER_LEG]
        )

        # fe_hip2 = angle_hip +- hip angle between ankle2 and knee1:
        hip_angle_ankle2_knee1 = qas.get_angle_between_points([pos_ankle2, self.pos_hip, self.pos_knee1])
        self.fe_hip2 = angle_hip + np.sign(midpoint_x) * hip_angle_ankle2_knee1

        # update fe_knee2:
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2

        # lift toes as much as possible:
        self.fe_ankle2 = MAX_ANKLE_FLEXION

        # Set hip_aa to average of start and end pose:
        end_pose = Pose(self.all_joint_names)
        end_pose.solve_end_position(ankle_x, ankle_y, ankle_z, subgait_id)
        self.aa_hip1 = start_hip_aa1 * (1 - midpoint_fraction) + end_pose.aa_hip1 * midpoint_fraction
        self.aa_hip2 = start_hip_aa2 * (1 - midpoint_fraction) + end_pose.aa_hip2 * midpoint_fraction

        # return pose as list:
        return self.pose_left if (subgait_id == "left_swing") else self.pose_right

    def solve_end_position(
        self,
        ankle_x: float,
        ankle_y: float,
        ankle_z: float,
        subgait_id: str,
        hip_x_fraction: float = DEFAULT_HIP_X_FRACTION,
        default_knee_bend: float = DEFAULT_KNEE_BEND,
        reduce_df_front: bool = True,
        reduce_df_rear: bool = True,
    ) -> List[float]:
        """Solves inverse kinematics for the end position.

        First determines the location at which the hip should be.
        Next it solves the joint angles for both legs to meet the hip and ankle locations.
        Finally it reduces dorsiflexion if required and adds the side step.

        Args:
            ankle_x (float): the forward distance for the end position.
            ankle_y (float): the upward distance for the end position.
            ankle_z (float): the sideward distance for the end position.
            subgait_id (str): either 'left_swing' or 'right_swing', defines which leg is the swing leg.
            hip_x_fraction (float): the fraction between the two feet (forward) at which the hip is desired.
            default_knee_bend (float): the default bending of the knee for a straight leg.
            reduce_df_front (bool): whether to reduce dorsiflexion for swing leg.
            reduce_df_rear (bool): whether to reduce dorsiflexion for stance leg.

        Returns:
            List[float]: a list of all the joint angles to perform the desired mid position.
        """
        # Reset:
        self.reset_to_zero_pose()

        # Set parameters:
        self.ankle_x = ankle_x
        self.ankle_y = ankle_y
        self.hip_x_fraction = hip_x_fraction
        self.knee_bend = default_knee_bend

        # Determine hip y-location:
        if ankle_y > 0:
            if hip_x_fraction >= 0.5:
                hip_y = np.sqrt(self.max_leg_length ** 2 - (self.hip_x) ** 2)
            else:
                hip_y = np.sqrt(self.max_leg_length ** 2 - (ankle_x - self.hip_x) ** 2)
        else:
            hip_y = min(
                ankle_y + np.sqrt(self.max_leg_length ** 2 - (ankle_x - self.hip_x) ** 2),
                np.sqrt(self.max_leg_length ** 2 - (self.hip_x) ** 2),
            )

        # Define hip and ankle locations:
        pos_hip = np.array([self.hip_x, hip_y])
        pos_ankle1 = np.array([0, 0])
        pos_ankle2 = np.array([ankle_x, ankle_y])

        # Solve legs without constraints:
        self.solve_leg(pos_hip, pos_ankle1, "rear")
        self.solve_leg(pos_hip, pos_ankle2, "front")

        # Reduce dorsi flexion to meet constraints:
        if reduce_df_front and self.fe_ankle2 > MAX_ANKLE_FLEXION:
            self.reduce_swing_dorsi_flexion()
            if self.pos_hip[0] < self.pos_ankle1[0]:
                self.keep_hip_above_rear_ankle()

        if reduce_df_rear and self.fe_ankle1 > MAX_ANKLE_FLEXION:
            self.reduce_stance_dorsi_flexion()

        # Apply side_step, hard_coded to default feet distance for now:
        self.perform_side_step(abs(ankle_y), abs(ankle_z))

        # Create a list of the pose:
        pose_list = self.pose_left if (subgait_id == "left_swing") else self.pose_right

        # Perform a limit check and raise error if limit is exceeded:
        errors = check_on_limits(self.all_joint_names, pose_list)
        if errors:
            for error in errors:
                raise ValueError(error)

        # return pose as list:
        return pose_list


# Static methods:
def rot(t: float) -> np.array:
    """Gives the rotation matrix for a given rotation.

    Returns the 2D rotation matrix R to rotate a vector with rotation t (in rad), so that::

        ⎡x'⎤ ⎽ ⎡cos(t) -sin(t)⎤⎡x⎤
        ⎣y'⎦ ⎺ ⎣sin(t)  cos(t)⎦⎣y⎦

    A positive value of t results in a anti-clockwise rotation around the origin.

    Args:
        t (float): desired rotation (theta) of a vector.

    Returns:
        np.array: a 2x2 numpy array representing the rotation matrix.
    """
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])


def check_on_limits(all_joint_names: List[str], pose_list: List[float]) -> List:
    """Checks all joints limits of the current pose and create error messages for exceeding limits.

    Args:
        all_joint_names (List[str]): list containing all joint names in alphabetical order.
        pose_list (List[float]): A list of joints poses in alphabetical order.
    """
    errors = []
    joint_pose_dict = {}
    # TODO: add global enum for all joint names
    for i, joint_name in enumerate(all_joint_names):
        joint_pose_dict[joint_name] = pose_list[i]

    for joint_name in JOINT_NAMES:
        if joint_pose_dict[joint_name] < JOINT_LIMITS[joint_name].lower:
            errors.append(
                "IK solver found a solution with joint "
                + joint_name
                + " "
                + str(round(JOINT_LIMITS[joint_name].lower - joint_pose_dict[joint_name], 3))
                + " rad below lower limit"
            )
        elif joint_pose_dict[joint_name] > JOINT_LIMITS[joint_name].upper:
            errors.append(
                "IK solver found a solution with joint "
                + joint_name
                + " "
                + str(round(joint_pose_dict[joint_name] - JOINT_LIMITS[joint_name].upper, 3))
                + " rad above upper limit"
            )

    return errors


def make_plot(pose: Pose):
    """Makes a plot of the exo by first calculating the joint positions and then plotting them.

    This method is only used for debugging reasons.

    Args:
        pose (Pose): The pose to plot.
    """
    positions = pose.calculate_joint_positions()
    positions_x = [pos[0] for pos in positions]
    positions_y = [pos[1] for pos in positions]

    plt.figure(1)
    plt.plot(positions_x, positions_y, ".-")
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()

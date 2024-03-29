"""Author: Jelmer de Wolde, MVII."""

import numpy as np

import march_goniometric_ik_solver.triangle_angle_solver as tas
import march_goniometric_ik_solver.quadrilateral_angle_solver as qas

from typing import List, Dict, Union, Optional
from march_goniometric_ik_solver.ik_solver_parameters import IKSolverParameters
from march_utility.exceptions.gait_exceptions import PositionSoftLimitError
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
JOINT_NAMES = get_joint_names_from_urdf(return_fixed_joints=True)

# Create a dictionary of joint limits:
JOINT_LIMITS = {}

for name in JOINT_NAMES:
    JOINT_LIMITS[name] = get_limits_robot_from_urdf_for_inverse_kinematics(name)

# Constants:
LENGTH_FOOT = 0.10  # m
DEFAULT_FOOT_DISTANCE = LENGTH_HIP_AA * 2 + LENGTH_HIP_BASE

ANKLE_ZERO_ANGLE = np.pi / 2  # rad
KNEE_ZERO_ANGLE = np.pi  # rad
HIP_ZERO_ANGLE = np.pi  # rad

# Default parameters:
DEFAULT_PARAMETERS = IKSolverParameters()


class Pose:
    """Pose class used for inverse kinematic calculations.

    Used to solve inverse kinematics for a desired end_position or mid_position of the foot.
    The class contains the joint_angles and the foot_rotation of the rear foot (in case of a toe-off)
    Solving can be done for the left or right foot, therefore this class uses the definition of 1 or 2
    for the joints, where 1 is the rear leg and 2 the front leg.
    Positive defined are: ankle dorsi-flexion, hip abduction, hip flexion, knee flexion.

    Args:
        parameters (IKSolverParameters): dataclass containing reconfigurable parameters
        pose (List[float]): List of the joint angles for the pose.
        leg1 (str): Leg that is the stance leg (leg1) of the starting position. For a right_swing, should be 'left'.

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

    def __init__(
        self,
        parameters: IKSolverParameters = DEFAULT_PARAMETERS,
        pose: Union[List[float], None] = None,
        leg1: str = "left",
    ) -> None:
        self._parameters = parameters

        self._max_ankle_dorsi_flexion = (
            min(JOINT_LIMITS["left_ankle"].upper, JOINT_LIMITS["right_ankle"].upper)
            - self._parameters.ankle_buffer_radians
        )
        self._max_ankle_plantar_flexion = (
            max(JOINT_LIMITS["left_ankle"].lower, JOINT_LIMITS["right_ankle"].lower)
            + self._parameters.ankle_buffer_radians
        )
        self._max_hip_extension = (
            max(JOINT_LIMITS["left_hip_fe"].lower, JOINT_LIMITS["right_hip_fe"].lower)
            + self._parameters.hip_buffer_radians
        )
        self._max_hip_abduction = (
            min(JOINT_LIMITS["left_hip_aa"].upper, JOINT_LIMITS["right_hip_aa"].upper)
            - self._parameters.hip_buffer_radians
        )
        self._max_hip_adduction = (
            max(JOINT_LIMITS["left_hip_aa"].lower, JOINT_LIMITS["right_hip_aa"].lower)
            + self._parameters.hip_buffer_radians
        )
        self.aa_hip1 = self.aa_hip2 = -0.05

        if pose is None:
            angle_ankle, angle_hip, angle_knee = self.leg_length_angles(self.max_leg_length)
            self.fe_ankle1 = self.fe_ankle2 = angle_ankle
            self.fe_hip1 = self.fe_hip2 = angle_hip
            self.fe_knee1 = self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee
            self.aa_hip1 = self.aa_hip2 = -0.05
        else:
            self.aa_hip1 = self.aa_hip2 = -0.05
            if leg1 == "left":
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
                self.aa_hip1 = self.aa_hip2 = -0.05
            elif leg1 == "right":
                (
                    self.fe_ankle2,
                    self.aa_hip2,
                    self.fe_hip2,
                    self.fe_knee2,
                    self.fe_ankle1,
                    self.aa_hip1,
                    self.fe_hip1,
                    self.fe_knee1,
                ) = pose
                self.aa_hip1 = self.aa_hip2 = -0.05
            else:
                raise ValueError(f"Invalid input {leg1}, should be 'right' or 'left'.")
        self.rot_foot1 = 0

    def reset_to_zero_pose(self) -> None:
        """Reset the post to the default zero pose."""
        self.__init__(self._parameters)

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

    def calculate_joint_positions(self) -> Dict[str, np.ndarray]:
        """Calculates the joint positions for a given pose as a chain from rear toes (toes1) to front toes (toes2).

        If a positive angle represents an anti-clockwise rotation, the angle variable is positive in the rot function.
        If a positive angle represents a clockwise rotation, the angle variable is negative in the rot function.
        The vectors (all np.array's) do always describe the translation when no rotation is applied.
        The rot_total matrix expands every step, since every joint location depends on all previous joint angles in the
        chain.

        Returns:
            Dict[str, np.ndarray[float]]: returns a dictionary of all joint positions.
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

        # return all joint positions:
        return {
            "pos_toes1": pos_toes1,
            "pos_ankle1": pos_ankle1,
            "pos_knee1": pos_knee1,
            "pos_hip": pos_hip,
            "pos_knee2": pos_knee2,
            "pos_ankle2": pos_ankle2,
            "pos_toes2": pos_toes2,
        }

    @property
    def pos_toes1(self) -> np.ndarray:
        """np.ndarray[float]. Calculates  [x. y] position of the stance leg toes."""
        return self.calculate_joint_positions()["pos_toes1"]

    @property
    def pos_ankle1(self) -> np.ndarray:
        """np.ndarray[float]. Calculates [x. y] position of the stance leg ankle."""
        return self.calculate_joint_positions()["pos_ankle1"]

    @property
    def pos_knee1(self) -> np.ndarray:
        """np.ndarray[float]. Calculates [x. y] position of the stance leg knee."""
        return self.calculate_joint_positions()["pos_knee1"]

    @property
    def pos_hip(self) -> np.ndarray:
        """np.ndarray[float]. Calculates [x. y] position of the hip."""
        return self.calculate_joint_positions()["pos_hip"]

    @property
    def pos_knee2(self) -> np.ndarray:
        """np.ndarray[float]. Calculates [x. y] position of the swing leg knee."""
        return self.calculate_joint_positions()["pos_knee2"]

    @property
    def pos_ankle2(self) -> np.ndarray:
        """np.ndarray[float]. Calculates [x. y] position of the swing leg ankle."""
        return self.calculate_joint_positions()["pos_ankle2"]

    @property
    def pos_toes2(self) -> np.ndarray:
        """np.ndarray[float]. Calculates [x. y] position of the swing leg toes."""
        return self.calculate_joint_positions()["pos_toes2"]

    @property
    def point_below_hip(self) -> np.ndarray:
        """np.ndarray[float]. Returns a ground point below the hip."""
        return np.array([self.pos_hip[0], 0])

    @property
    def point_right_to_ankle2(self) -> np.ndarray:
        """np.ndarray[float]. Returns a point right from ankle at distance LENGTH_FOOT."""
        return np.array([self.pos_ankle2[0] + LENGTH_FOOT, self.pos_ankle2[1]])

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
            knee_angle = self._parameters.default_knee_bend_radians

        pos_hip = pos_knee + rot(knee_angle) @ np.array([0, LENGTH_UPPER_LEG])
        return np.linalg.norm(pos_hip - pos_ankle)

    @property
    def ankle_limit_pos_hip(self) -> np.ndarray:
        """np.ndarray[float]. Returns the hip position when the ankle is in max dorsi flexion."""
        pose = Pose(self._parameters)
        pose.fe_ankle1 = self._max_ankle_dorsi_flexion
        return pose.pos_hip

    @property
    def ankle_limit_toes_knee_distance(self) -> float:
        """Returns the distance between knee and toes when the ankle is in max dorsi flexion."""
        pose = Pose(self._parameters)
        pose.fe_ankle1 = self._max_ankle_dorsi_flexion
        return np.linalg.norm(pose.pos_knee1 - pose.pos_toes1)

    @property
    def ankle_limit_toes_hip_distance(self) -> float:
        """Returns the distance between hip and toes when the ankle is in max dorsi flexion."""
        pose = Pose(self._parameters)
        pose.fe_ankle1 = self._max_ankle_dorsi_flexion
        return np.linalg.norm(pose.pos_hip - pose.pos_toes1)

    @property
    def min_step_size_rotated_foot(self) -> float:
        """Returns the minimum step distance at which ankle2 can reach the ground with ankle1 at maximum dorsi flexion and no flat rear foot."""
        if self.ankle_limit_toes_hip_distance > self.max_leg_length:
            return LENGTH_FOOT + np.sqrt(self.ankle_limit_toes_hip_distance ** 2 - self.max_leg_length ** 2)
        else:
            return self.ankle_limit_pos_hip[0]

    @property
    def min_step_height_rotated_foot(self) -> float:
        """Returns the lowest possible ankle2 height which can be reached with ankle1 at maximum dorsi flexion and no flat rear foot."""
        long_side = self.ankle_limit_toes_hip_distance
        horizontal_side = self.ankle_x - LENGTH_FOOT
        if long_side >= horizontal_side:
            hip_height = np.sqrt(long_side ** 2 - horizontal_side ** 2)
        else:
            hip_height = 0.0
        return hip_height - self.max_leg_length

    @property
    def max_step_size_flat_foot(self) -> float:
        """Returns the maximum step distance with the rear foot flat on the ground."""
        return self.ankle_limit_pos_hip[0] * 2

    def leg_length_angles(self, leg_length: float) -> List[float]:
        """Returns the required angles in the triangle between ankle, hip and knee to meet the leg_length.

        Args:
            leg_length (float): the desired distance between ankle and hip.

        Returns:
            List[float]: returns the required angles in order: ankle, hip knee.
        """
        if leg_length < LENGTH_LEG:
            sides = [LENGTH_UPPER_LEG, LENGTH_LOWER_LEG, leg_length]
            angle_ankle, angled_hip, angle_knee = tas.get_angles_from_sides(sides)
            return [angle_ankle, angled_hip, angle_knee]
        else:
            return [0.0, 0.0, np.pi]

    def solve_leg(
        self,
        pos_hip: np.ndarray,
        pos_ankle: np.ndarray,
        leg: str,
    ) -> None:
        """Solve the joint angles for a given leg to have hip and ankle at given positions.

        Args:
            pos_hip (np.ndarray[float]): a 2D numpy array containing the position of the hip
            pos_ankle (np.ndarray[float]): a 2D numpy array containing the position of the ankle
            leg (str): defines for which leg it needs to solve, can be 'rear' or 'front'.
        """
        dist_ankle_hip = np.linalg.norm(pos_hip - pos_ankle)
        angle_ankle, angle_hip, angle_knee = self.leg_length_angles(dist_ankle_hip)

        base_angle = np.arcsin(abs(pos_ankle[0] - pos_hip[0]) / dist_ankle_hip)

        # Note: np.signs are there for when hip is behind ankle (can occur in mid position)
        if leg == "rear":
            self.fe_ankle1 = np.sign(pos_hip[0] - pos_ankle[0]) * base_angle + angle_ankle
            self.fe_hip1 = np.sign(pos_ankle[0] - pos_hip[0]) * base_angle + angle_hip
            self.fe_knee1 = KNEE_ZERO_ANGLE - angle_knee

        elif leg == "front":
            self.fe_ankle2 = np.sign(pos_hip[0] - pos_ankle[0]) * base_angle + angle_ankle
            self.fe_hip2 = np.sign(pos_ankle[0] - pos_hip[0]) * base_angle + angle_hip

            self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee

        else:
            raise ValueError("Expected leg to be 'rear' or 'front'.")

    def reduce_swing_dorsi_flexion_update_pose(
        self, rotation_point: str, rotation: float, angle_knee2: float, angle_hip: float
    ) -> None:
        """Updates the pose based on a certain dorsi flexion reduction of the swing leg.

        Args:
            rotation_point (str): the point we rotate around to reduce dorsi flexion. This can be 'toes' or 'ankle'.
            rotation (float): the amount of rotation we apply to reduce dorsi flexion.
            angle_knee2 (float): the angle of knee2 calculated with the reduce_swing_dorsi_flexion_calculate_angles() method.
            angle_hip (float): the angle of hip calculated with the reduce_swing_dorsi_flexion_calculate_angles() method.
        """
        if rotation_point == "toes":
            self.rot_foot1 -= rotation
            pos_rot = self.pos_toes1
        elif rotation_point == "ankle":
            self.fe_ankle1 -= rotation
            pos_rot = self.pos_ankle1

        self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )
        self.fe_hip2 = angle_hip + np.sign(pos_rot[0] - self.pos_hip[0]) * qas.get_angle_between_points(
            [pos_rot, self.pos_hip, self.point_below_hip]
        )
        self.fe_knee2 = KNEE_ZERO_ANGLE - angle_knee2
        self.fe_ankle2 = ANKLE_ZERO_ANGLE - qas.get_angle_between_points(
            [self.pos_knee2, self.pos_ankle2, self.point_right_to_ankle2]
        )

    def reduce_swing_dorsi_flexion_calculate_angles(self, known_angle: float, rotation_point: str) -> List[float]:
        """Calculates the other angles for the quadrilateral used to reduce dorsi_flexion.

        Args:
            known_angle (float): the angle in the quadrilateral that is known/defined.
            rotation_point (str): the point we rotate around to reduce dorsi flexion. This can be 'toes' or 'ankle'.

        Returns:
            List[float]: returns the angles for the used quadrilateral.
        """
        # Define desired angle_ankle2 and determine other angles in quadrilateral:
        dist_toes1_ankle2 = np.linalg.norm(self.pos_toes1 - self.pos_ankle2)
        dist_ankle1_ankle2 = np.linalg.norm(self.pos_ankle1 - self.pos_ankle2)

        if rotation_point == "toes":
            sides = [
                self.ankle_limit_toes_hip_distance,
                dist_toes1_ankle2,
                LENGTH_LOWER_LEG,
                LENGTH_UPPER_LEG,
            ]
        elif rotation_point == "ankle":
            sides = [
                self.max_leg_length,
                dist_ankle1_ankle2,
                LENGTH_LOWER_LEG,
                LENGTH_UPPER_LEG,
            ]

        angle_rotation_point, angle_ankle2, angle_knee2, angle_hip = qas.solve_quadritlateral(sides, known_angle)
        return [angle_rotation_point, angle_ankle2, angle_knee2, angle_hip]

    def reduce_swing_dorsi_flexion(self) -> None:
        """Calculates the pose after reducing the dorsiflexion for the swing leg.

        If the current pose has foot rotation, it creates a quadrilateral between toes1, ankle2, knee2, hip.
        If reducing is not (completely) possible with foot rotation, it creates a quadrilateral between ankle1, ankle2, knee2, hip.
        It reduces the angle at ankle2 so that dorsiflexion is within the limit.
        Next it defines the other angles in the quadrilateral with this new ankle angle.
        Finally it updates the pose with the reduced dorsiflexion.
        """
        # First try to reduce dorsi flexion with foot rotation:
        if self.rot_foot1 > 0:

            # Define desired angle_ankle2 and determine other angles in quadrilateral:
            reduction = self.fe_ankle2 - self._max_ankle_dorsi_flexion
            angle_ankle2 = qas.get_angle_between_points([self.pos_toes1, self.pos_ankle2, self.pos_knee2]) - reduction
            angle_toes1, angle_ankle2, angle_knee2, angle_hip = self.reduce_swing_dorsi_flexion_calculate_angles(
                known_angle=angle_ankle2, rotation_point="toes"
            )

            # If we can compensate everything with foot rotation, we update the pose:
            angle_toes1_before = qas.get_angle_between_points([self.pos_ankle2, self.pos_toes1, self.pos_hip])
            rotation = angle_toes1 - angle_toes1_before
            if rotation < self.rot_foot1:
                self.reduce_swing_dorsi_flexion_update_pose(
                    "toes", rotation=rotation, angle_knee2=angle_knee2, angle_hip=angle_hip
                )

            # Otherwise we reset pose to have zero foot rotation and compensate the rest with ankle rotation:
            else:
                self.reset_to_zero_pose()
                self.fe_ankle1 = self._max_ankle_dorsi_flexion
                self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
                    [self.pos_knee1, self.pos_hip, self.point_below_hip]
                )
                self.solve_leg(self.pos_hip, np.array([self.ankle_x, self.ankle_y]), "front")

        # If it was not possible to reduce (everything) with foot rotation, we will reduce with ankle rotation:
        if self.fe_ankle2 > self._max_ankle_dorsi_flexion:

            # Define desired angle_ankle2 and determine other angles in quadrilateral:
            reduction = self.fe_ankle2 - self._max_ankle_dorsi_flexion
            angle_ankle2 = qas.get_angle_between_points([self.pos_ankle1, self.pos_ankle2, self.pos_knee2]) - reduction
            angle_ankle1, angle_ankle2, angle_knee2, angle_hip = self.reduce_swing_dorsi_flexion_calculate_angles(
                known_angle=angle_ankle2, rotation_point="ankle"
            )

            # Update the pose:
            angle_ankle1_before = qas.get_angle_between_points([self.pos_ankle2, self.pos_ankle1, self.pos_hip])
            rotation = angle_ankle1 - angle_ankle1_before
            self.reduce_swing_dorsi_flexion_update_pose(
                "ankle", rotation=rotation, angle_knee2=angle_knee2, angle_hip=angle_hip
            )

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
        self.fe_ankle2 = self._max_ankle_dorsi_flexion

    def reduce_hip_extension(self) -> None:
        """Reduces hip extension of one while keeping the same step size, by adding flexion to the other leg."""
        if self.fe_hip1 < self._max_hip_extension:
            reduction = self._max_hip_extension - self.fe_hip1
            self.fe_hip1 += reduction
            self.fe_hip2 += reduction
        if self.fe_hip2 < self._max_hip_extension:
            reduction = self._max_hip_extension - self.fe_hip2
            self.fe_hip1 += reduction
            self.fe_hip2 += reduction

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

    def solve_mid_position_with_flat_stance_foot(self, hip_mid_x: float, pos_ankle: np.ndarray):
        """Solves the required pose for a midpoint location small enough to keep the stance foot flat on the ground."""
        max_height_swing_leg = pos_ankle[1] + np.sqrt(self.max_leg_length ** 2 - (pos_ankle[0] - hip_mid_x) ** 2)
        max_height_stance_leg = np.sqrt(self.max_leg_length ** 2 - (hip_mid_x) ** 2)
        hip_mid_y = min(max_height_swing_leg, max_height_stance_leg)
        hip_mid = np.array([hip_mid_x, hip_mid_y])
        self.solve_leg(hip_mid, self.pos_ankle1, "rear")
        self.solve_leg(hip_mid, pos_ankle, "front")

    def solve_mid_position_with_rotated_stance_foot(self, hip_mid_x: float, pos_ankle: np.ndarray):
        """Solves the required pose for midpoint location that requires foot rotation of the stance foot to reach the desired location."""
        hip_mid_y = np.sqrt(self.ankle_limit_toes_hip_distance ** 2 - (hip_mid_x - self.pos_toes1[0]) ** 2)
        hip_mid = np.array([hip_mid_x, hip_mid_y])
        self.rot_foot1 = qas.get_angle_between_points([self.ankle_limit_pos_hip, self.pos_toes1, hip_mid])
        self.fe_ankle1 = self._max_ankle_dorsi_flexion
        self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )
        self.solve_leg(hip_mid, pos_ankle, "front")

    def solve_mid_position(
        self,
        next_pose: "Pose",
        frac: float,
        ankle_y: float,
        subgait_id: str,
    ) -> List[float]:
        """Solves inverse kinematics for the middle position.

        Assumes that the stance leg is straight with a knee flexion of self._parameters.default_knee_bend_radians.
        First it calculates the desired foot location using end position, midpoint fraction and midpoint height.
        Next it resets to zero pose and calculates the required angles for the swing leg to reach the calculated midpoint.

        Args:
            next_pose (Pose): the next pose to move to.
            frac (float): the fraction of the step at which the mid position should be.
            ankle_y (float): the desired height  of the ankle for the mid position.
            subgait_id (str): either 'left_swing' or 'right_swing', defines which leg is the swing leg.

        Returns:
            List[float]: a list of all the joint angles to perform the desired mid position.
        """
        pos_ankle = np.array([self.get_ankle_mid_x(frac, next_pose), ankle_y])

        # Store current hip_aa to calculate hip_aa of midpoint later:
        current_hip_aa_1, current_hip_aa_2 = self.aa_hip1, self.aa_hip2

        # Translate current hip and ankle location to new reference frame:
        hip_current = self.pos_hip - self.pos_ankle2

        # Define hip_mid_x as the given fraction between current hip location and the hip location in next pose:
        hip_mid_x = (1 - frac) * hip_current[0] + frac * next_pose.pos_hip[0]

        # Solve the mid pose:
        self.reset_to_zero_pose()
        if hip_mid_x <= self.ankle_limit_pos_hip[0]:
            self.solve_mid_position_with_flat_stance_foot(hip_mid_x, pos_ankle)
        else:
            self.solve_mid_position_with_rotated_stance_foot(hip_mid_x, pos_ankle)

        # Apply the desired rotation of the upper body:
        self.fe_hip2 += self._parameters.upper_body_front_rotation_radians
        self.fe_hip1 += self._parameters.upper_body_front_rotation_radians

        # Lift toes of swing leg as much as possible:
        self.fe_ankle2 = self._max_ankle_dorsi_flexion

        # Reduce hip extension if necessary:
        self.reduce_hip_extension()

        # Add hip_swing or set hip_aa to average of start and end pose:
        if self._parameters.hip_swing and 0 < self._parameters.hip_swing_fraction < 1:
            max_hip_swing = min(abs(self._max_hip_abduction), abs(self._max_hip_adduction))
            self.aa_hip1 = -max_hip_swing * self._parameters.hip_swing_fraction
        else:
            self.aa_hip1 = current_hip_aa_1 * (1 - frac) + next_pose.aa_hip1 * frac
            self.aa_hip2 = current_hip_aa_2 * (1 - frac) + next_pose.aa_hip2 * frac

        # Create a list of the pose:
        pose_list = self.pose_left if (subgait_id == "left_swing") else self.pose_right

        # Perform a limit check and raise error if limit is exceeded:
        check_on_limits(pose_list)

        # return pose as list:
        return pose_list

    def get_ankle_mid_x(self, hip_frac: float, next_pose: "Pose"):
        """Calculates the ankle x position relative to the stance leg.

        Arguments:
            hip_frac: fraction of the step at which the hip is placed
            next_pose (Pose): pose of the end position

        Returns:
            float: swing leg ankle x position relative to the stance leg
        """
        frac_relative_to_stance_leg = (
            hip_frac - self._parameters.middle_point_fraction
        ) / self._parameters.middle_point_fraction
        shift_ankle_x_relative_to_stance_leg = (-self.pos_ankle2[0] + next_pose.pos_ankle2[0]) / 2
        step_length = self.pos_ankle2[0] + next_pose.pos_ankle2[0]
        relative_fraction = 1 - (1 - self._parameters.base_number ** (1 - abs(frac_relative_to_stance_leg))) / (
            1 - self._parameters.base_number
        )
        return (
            np.sign(frac_relative_to_stance_leg) * relative_fraction * (step_length / 2)
            + shift_ankle_x_relative_to_stance_leg
        )

    def step_with_flat_stance_foot(self) -> None:
        """Solves the required pose for a step small enough to keep the stance foot flat on the ground."""
        hip_y = np.sqrt(self.max_leg_length ** 2 - self.hip_x ** 2)
        pos_hip = np.array([self.hip_x, hip_y])
        self.solve_leg(pos_hip, self.desired_pos_ankle1, "rear")
        self.solve_leg(pos_hip, self.desired_pos_ankle2, "front")

    def set_hip_intersection_of_circles(self) -> None:
        """Determines the desired hip position using the intersection of circles method."""
        pos_hip_solutions = qas.get_intersection_of_circles(
            self.pos_toes1,
            np.array([self.ankle_x, 0.0]),
            self.ankle_limit_toes_hip_distance,
            self.max_leg_length,
        )

        if pos_hip_solutions[0] is not None:
            self.desired_pos_hip = abs(pos_hip_solutions[0])
        else:
            raise ValueError("Expected a solution for the hip that is not None.")

    def set_hip_linearly_scaled(self) -> None:
        """Determines the desired hip position as a percentage between the two other methods.

        It linearly scales the hip position between the hip position for the largest possible flat foot step and
        the hip position for the smallest possible step using the intersection_of_circles method.
        This linearly scaling results in a fluent transition between the two previous methods.
        """
        hip_x = self.ankle_limit_pos_hip[0] + (self.ankle_x - self.max_step_size_flat_foot) / (
            self.min_step_size_rotated_foot - self.max_step_size_flat_foot
        ) * (self.min_step_size_rotated_foot - self.ankle_limit_pos_hip[0])
        hip_y = np.sqrt(self.ankle_limit_toes_hip_distance ** 2 - (hip_x - self.pos_toes1[0]) ** 2)
        self.desired_pos_hip = np.array([hip_x, hip_y])

        # If the required ankle height is larger than the desired height, we will use the required ankle height:
        leg_height = np.sqrt(self.max_leg_length ** 2 - (self.ankle_x - hip_x) ** 2)
        ankle_y_required = hip_y - leg_height
        if ankle_y_required > self.ankle_y:
            self.ankle_y = ankle_y_required

    def step_with_rotated_stance_foot(self) -> None:
        """Solves the required pose for a step that requires foot rotation of the stance foot to reach the desired location."""
        self.rot_foot1 = qas.get_angle_between_points([self.ankle_limit_pos_hip, self.pos_toes1, self.desired_pos_hip])
        self.fe_ankle1 = self._max_ankle_dorsi_flexion
        self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )
        self.solve_leg(self.desired_pos_hip, np.array([self.ankle_x, self.ankle_y]), "front")

    def step_down_with_rotated_stance_foot(self) -> None:
        """Solves the required pose for a step down that requires foot rotation of the stance foot to reach the desired location."""
        pos_hip_solutions = qas.get_intersection_of_circles(
            self.pos_toes1,
            self.desired_pos_ankle2,
            self.ankle_limit_toes_hip_distance,
            self.max_leg_length,
        )

        if pos_hip_solutions[0] is not None:
            pos_hip = abs(pos_hip_solutions[0])
            if pos_hip[0] > self.ankle_limit_pos_hip[0]:
                self.rot_foot1 = qas.get_angle_between_points([self.ankle_limit_pos_hip, self.pos_toes1, pos_hip])
                self.fe_ankle1 = self._max_ankle_dorsi_flexion
                self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
                    [self.pos_knee1, self.pos_hip, self.point_below_hip]
                )
            else:
                self.solve_leg(pos_hip, np.array([0, 0]), "rear")

            self.solve_leg(pos_hip, self.desired_pos_ankle2, "front")
        else:
            raise ValueError("Expected a solution for the hip that is not None.")

    def get_knee1_position_for_hip_above_ankle2(self) -> Union[np.ndarray, None]:
        """Calculates the knee1 position for step down with the hip above ankle2.

        Returns:
            Union[np.ndarray[float], None]: Returns the position of knee1, or None if there is no solution possible.
        """
        self.desired_pos_hip = self.desired_pos_ankle2 + np.array([0.0, self.max_leg_length])

        # pos_knee1:
        return qas.get_intersection_of_circles(
            self.pos_toes1,
            self.desired_pos_hip,
            self.ankle_limit_toes_knee_distance,
            LENGTH_UPPER_LEG,
        )[1]

    def step_down_with_hip_above_ankle2(self, pos_knee1: np.ndarray) -> None:
        """Solves the required pose for a step down with the hip located above ankle2.

        Args:
            pos_knee1 (np.ndarray[float]): The required position of pos_knee1 to get the hip above ankle2.
        """
        self.fe_ankle1 = self._max_ankle_dorsi_flexion
        self.rot_foot1 = qas.get_angle_between_points([pos_knee1, self.pos_toes1, self.pos_knee1])
        self.fe_knee1 = KNEE_ZERO_ANGLE - qas.get_angle_between_points(
            [self.pos_ankle1, self.pos_knee1, self.desired_pos_hip]
        )
        self.fe_hip1 = np.sign(self.pos_knee1[0] - self.pos_hip[0]) * qas.get_angle_between_points(
            [self.pos_knee1, self.pos_hip, self.point_below_hip]
        )
        self.solve_leg(self.desired_pos_hip, self.desired_pos_ankle2, "front")

    def step_down_wih_triangle_method(self) -> None:
        """Solves the required pose for a step down by forming triangle between the two ankles and the hip."""
        hip_y = self.ankle_y + np.sqrt(self.max_leg_length ** 2 - (self.ankle_x - self.hip_x) ** 2)
        pos_hip = np.array([self.hip_x, hip_y])
        self.solve_leg(pos_hip, self.desired_pos_ankle1, "rear")
        self.solve_leg(pos_hip, self.desired_pos_ankle2, "front")

    def solve_end_position(
        self,
        ankle_x: float,
        ankle_y: float,
        ankle_z: float,
        subgait_id: str,
        hip_x_fraction: Optional[float] = None,
        default_knee_bend: Optional[float] = None,
        reduce_df_front: bool = True,
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

        Returns:
            List[float]: a list of all the joint angles to perform the desired mid position.
        """
        # Reset:
        self.reset_to_zero_pose()

        # Set parameters:
        self.ankle_x = ankle_x
        self.ankle_y = ankle_y
        self.hip_x_fraction = self._parameters.hip_x_fraction if hip_x_fraction is None else hip_x_fraction
        self.knee_bend = default_knee_bend if default_knee_bend else self._parameters.default_knee_bend_radians
        self.desired_pos_ankle1 = np.array([0, 0])
        self.desired_pos_ankle2 = np.array([self.ankle_x, self.ankle_y])

        # Solve pose:
        if self.ankle_y >= 0:
            if self.ankle_x <= self.max_step_size_flat_foot:
                self.step_with_flat_stance_foot()
            else:
                if self.ankle_x >= self.min_step_size_rotated_foot:
                    self.set_hip_intersection_of_circles()
                else:
                    self.set_hip_linearly_scaled()
                self.step_with_rotated_stance_foot()
        else:
            if self.ankle_x >= self.min_step_size_rotated_foot and self.ankle_y >= self.min_step_height_rotated_foot:
                self.step_down_with_rotated_stance_foot()
            else:
                pos_knee1 = self.get_knee1_position_for_hip_above_ankle2()

                if pos_knee1 is not None:
                    self.step_down_with_hip_above_ankle2(pos_knee1=pos_knee1)
                else:
                    self.step_down_wih_triangle_method()

        # Apply the desired rotation of the upper body:
        self.fe_hip2 += self._parameters.upper_body_front_rotation_radians
        self.fe_hip1 += self._parameters.upper_body_front_rotation_radians

        # Reduce ankle2 dorsi flexion to meet constraints:
        if reduce_df_front and self.fe_ankle2 > self._max_ankle_dorsi_flexion:
            self.reduce_swing_dorsi_flexion()

            if self.pos_hip[0] < self.pos_ankle1[0]:
                self.keep_hip_above_rear_ankle()

        if self.fe_ankle2 < self._max_ankle_plantar_flexion:
            self.fe_ankle2 = self._max_ankle_plantar_flexion

        # Reduce hip extension if necessary:
        self.reduce_hip_extension()

        # Apply side_step, hard_coded to default feet distance for now:

        # Raise the toes when desired:
        if self._parameters.dorsiflexion_at_end_position_radians != 0:
            self.fe_ankle2 = min(
                max(self._parameters.dorsiflexion_at_end_position_radians, self.fe_ankle2),
                self._max_ankle_dorsi_flexion,
            )

        # Create a list of the pose:
        pose_list = self.pose_left if (subgait_id == "left_swing") else self.pose_right

        # Perform a limit check and raise error if limit is exceeded:
        check_on_limits(pose_list)

        # return pose as list:
        return pose_list


# Static methods:
def rot(t: float) -> np.ndarray:
    """Gives the rotation matrix for a given rotation.

    Returns the 2D rotation matrix R to rotate a vector with rotation t (in rad), so that::

        ⎡x'⎤ ⎽ ⎡cos(t) -sin(t)⎤⎡x⎤
        ⎣y'⎦ ⎺ ⎣sin(t)  cos(t)⎦⎣y⎦

    A positive value of t results in a anti-clockwise rotation around the origin.

    Args:
        t (float): desired rotation (theta) of a vector.

    Returns:
        np.ndarray[float]: a 2x2 numpy array representing the rotation matrix.
    """
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])


def check_on_limits(pose_list: List[float]) -> None:
    """Checks all joints limits of the current pose and create error messages for exceeding limits.

    Args:
        all_joint_names (List[str]): list containing all joint names in alphabetical order.
        pose_list (List[float]): A list of joints poses in alphabetical order.
    """
    joint_pose_dict = {}
    # TODO: add global enum for all joint names
    for i, joint_name in enumerate(JOINT_NAMES):
        joint_pose_dict[joint_name] = pose_list[i]

    for joint_name in JOINT_NAMES:
        if (joint_pose_dict[joint_name] < JOINT_LIMITS[joint_name].lower) or (
            joint_pose_dict[joint_name] > JOINT_LIMITS[joint_name].upper
        ):
            raise PositionSoftLimitError(
                joint_name, joint_pose_dict[joint_name], JOINT_LIMITS[joint_name].lower, JOINT_LIMITS[joint_name].upper
            )

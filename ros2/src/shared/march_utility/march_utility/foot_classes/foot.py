"""
This module contains the foot class.

This class captures the state of a foot at a specific time. This is used for
creating gaits based on foot positions.
"""

from __future__ import annotations

from math import acos, asin, atan, atan2, cos, pi, sin, sqrt
from typing import Tuple

from march_utility.exceptions.gait_exceptions import SubgaitInterpolationError
from march_utility.exceptions.general_exceptions import SideSpecificationError
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.side import Side
from march_utility.utilities.utility_functions import (
    get_joint_names_for_inverse_kinematics,
)
from march_utility.utilities.utility_functions import (
    get_lengths_robot_for_inverse_kinematics,
    weighted_average_vectors,
)
from march_utility.utilities.vector_3d import Vector3d

VELOCITY_SCALE_FACTOR = 0.001
JOINT_NAMES_IK = get_joint_names_for_inverse_kinematics()
MID_CALCULATION_PRECISION_DIGITS = 10
ALLOWABLE_OVERSHOOT_FOOT_POSITION = 0.001


class Foot(object):
    """Class for capturing the state (position and possible velocity) of a foot."""

    def __init__(self, foot_side: Side, position: Vector3d, velocity: Vector3d) -> None:
        """Create a Foot object, position and velocity are both Vector3d objects."""
        self.position: Vector3d = position
        self.velocity: Vector3d = velocity
        if foot_side != Side.left and foot_side != Side.right:
            raise SideSpecificationError(foot_side)
        self.foot_side = Side(foot_side)

    def add_foot_velocity_from_next_state(self, next_state: Foot) -> None:
        """Add the foot velocity to the foot state.

        This is done based on a given next state for the foot to be in.

        :param: next_state:
            A Foot object that specifies the foot location 1 / VELOCITY_SCALE_FACTOR
            seconds later.

        :return:
            The object with a velocity which is calculated based on the next state.
        """
        velocity = (next_state.position - self.position) / VELOCITY_SCALE_FACTOR
        self.velocity = velocity

    @classmethod
    def calculate_next_foot_position(cls, current_state: Foot) -> Foot:
        """Calculate the foot position a moment later given the current state.

        :param current_state:
            A Foot object with a velocity.

        :return:
            A Foot object with the position of the foot 1 / VELOCITY_SCALE_FACTOR
            seconds later.
        """
        next_position = current_state.position + (
            current_state.velocity * VELOCITY_SCALE_FACTOR
        )
        return cls(current_state.foot_side, next_position, current_state.velocity)

    @staticmethod
    def get_joint_states_from_foot_state(foot_state: Foot, time: float) -> dict:
        """Translate between feet_state and a list of setpoints.

        :param foot_state:
            A fully populated Foot object.
        :param time:
            The time of the Foot state and resulting setpoints.

        :return:
            A dictionary of setpoints, the foot location and velocity of
            which corresponds with the feet_state.
        """
        joint_states = Foot.calculate_joint_angles_from_foot_position(
            foot_state.position, foot_state.foot_side, time
        )

        # Find the joint angles a moment later using the foot position a
        # moment later use this together with the current joint angles to
        # calculate the joint velocity
        next_position = Foot.calculate_next_foot_position(foot_state)
        next_joint_positions = Foot.calculate_joint_angles_from_foot_position(
            next_position.position,
            next_position.foot_side,
            time + Duration(seconds=VELOCITY_SCALE_FACTOR),
        )

        for joint in JOINT_NAMES_IK:
            if joint in joint_states and joint in next_joint_positions:
                joint_states[joint].add_joint_velocity_from_next_angle(
                    next_joint_positions[joint]
                )

        return joint_states

    @staticmethod
    def calculate_joint_angles_from_foot_position(
        foot_position: Vector3d, foot_side: Side, time: Duration
    ) -> dict:
        """Calculate the angles of the joints corresponding to a certain foot position.

        More information on the calculations of the haa, hfe and kfe angles,
        as well as on the velocity calculations can be found at
        https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics.
        This function assumes that the desired z position of the foot is positive.

        :param foot_position:
            A Vecor3d object which specifies the desired position of the foot.
        :param foot_side: A string which specifies to which side the
            foot_position belongs and thus which joint angles should be computed.
        :param time:
            The time of the foot_position and the resulting setpoints.

        :return:
            A dictionary of Setpoints for each joint on the requested side with
            the correct angle at the provided time.
        """
        if foot_side != Side.left and foot_side != Side.right:
            raise SideSpecificationError(foot_side)
        # Get relevant lengths from robot model, ul = upper leg etc.
        # see get_lengths_robot_for_inverse_kinematics() and unpack desired position
        [ul, ll, hl, ph, base] = get_lengths_robot_for_inverse_kinematics(foot_side)
        x_position = foot_position.x
        y_position = foot_position.y
        z_position = foot_position.z

        # Change y positive direction to the desired foot, change origin to
        # pivot of haa joint, for ease of calculation.
        if foot_side == Side.left:
            y_position = -(y_position + base / 2.0)
        else:
            y_position = y_position - base / 2.0

        # First calculate the haa angle. This calculation assumes that pos_z > 0
        haa = Foot.calculate_haa_angle(z_position, y_position, ph)

        # Once the haa angle is known, transform the desired x and z position to
        # arrive at an easier system to calculate the hfe and kfe angles
        (
            transformed_x,
            transformed_z,
            transformed_distance_to_origin,
        ) = Foot.transform_desired_position(x_position, z_position, y_position, ph, hl)

        # If the desired foot location is too far out, trow an error
        if transformed_distance_to_origin > ll + ul + ALLOWABLE_OVERSHOOT_FOOT_POSITION:
            raise SubgaitInterpolationError(
                f"The desired {foot_side} foot position, (x, y, z) = ({x_position}, "
                f"{y_position}, {z_position}), is out of reach. Transformed coordinates "
                f"are (x', z') = ({transformed_x}, {transformed_z}) with haa angle {haa}."
                f"Distance to origin {transformed_distance_to_origin}."
            )

        hfe, kfe = Foot.calculate_hfe_kfe_angles(
            transformed_x, transformed_z, ul, ll, transformed_distance_to_origin
        )

        return {
            foot_side.value + "_hip_aa": Setpoint(time, haa),
            foot_side.value + "_hip_fe": Setpoint(time, hfe),
            foot_side.value + "_knee": Setpoint(time, kfe),
        }

    @staticmethod
    def transform_desired_position(
        x_position: float, z_position: float, y_position: float, ph: float, hl: float
    ) -> Tuple[float, float, float]:
        """Transform the desired position to an easier coordinate system for the hfe kfe calculation."""
        transformed_x = round(x_position - hl, MID_CALCULATION_PRECISION_DIGITS)
        transformed_z = round(
            sqrt(-ph * ph + y_position * y_position + z_position * z_position),
            MID_CALCULATION_PRECISION_DIGITS,
        )
        transformed_distance_to_origin = sqrt(
            transformed_x * transformed_x + transformed_z * transformed_z
        )
        return transformed_x, transformed_z, transformed_distance_to_origin

    @staticmethod
    def calculate_hfe_kfe_angles(
        transformed_x: float,
        transformed_z: float,
        ul: float,
        ll: float,
        transformed_distance_to_origin: float,
    ) -> Tuple[float, float]:
        """Figure out how to calculate the hfe and kfe angles and do the calculations."""
        # If the desired foot location is just beyond what is reachable, (due to rounding errors perhaps),
        # do a calculation which assumes the leg is stretched
        if (
            ll + ul
            <= transformed_distance_to_origin
            <= ll + ul + ALLOWABLE_OVERSHOOT_FOOT_POSITION
        ):
            hfe = Foot.calculate_hfe_angle_straight_leg(transformed_x, transformed_z)
            kfe = 0
        # If neither is the case, do the normal hfe kfe calculation
        else:
            hfe, kfe = Foot.calculate_hfe_kfe_angles_default_situation(
                transformed_x, transformed_z, ul, ll
            )
        return hfe, kfe

    @staticmethod
    def calculate_hfe_angle_straight_leg(
        transformed_x: float, transformed_z: float
    ) -> float:
        """Calculate the hfe angle of a straight leg."""
        return atan2(transformed_x, transformed_z)

    @staticmethod
    def calculate_haa_angle(
        z_position: float, y_position: float, pelvis_hip_length: float
    ) -> float:
        """Calculate the haa angle of the exoskeleton.

        This is done based on a given desired y and z position of the exoskeleton.
        More information on the calculation is found at
        https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics

        :param z_position:
            The desired z-position of the foot
        :param y_position:
            The desired y-position of the foot
        :param pelvis_hip_length:
            The length from the pelvis to the hip_aa, which is the haa arm

        :return:
            The hip_aa joint angle needed for the foot to reach the given position
        """
        if z_position <= 0:
            raise SubgaitInterpolationError(
                "Desired z position of the foot is not positive, "
                "current haa calculation is not capable to deal with this"
            )

        if y_position != 0:
            slope_foot_to_origin = z_position / y_position
            angle_foot_to_origin = atan(slope_foot_to_origin)
            if y_position > 0:
                haa = (
                    acos(
                        pelvis_hip_length
                        / sqrt(z_position * z_position + y_position * y_position)
                    )
                    - angle_foot_to_origin
                )
            else:
                haa = (
                    acos(
                        pelvis_hip_length
                        / sqrt(z_position * z_position + y_position * y_position)
                    )
                    - pi
                    - angle_foot_to_origin
                )
        else:
            angle_foot_to_origin = pi / 2
            haa = (
                acos(
                    pelvis_hip_length
                    / sqrt(z_position * z_position + y_position * y_position)
                )
                - angle_foot_to_origin
            )

        return haa

    @staticmethod
    def calculate_hfe_kfe_angles_default_situation(
        transformed_x: float, transformed_z: float, upper_leg: float, lower_leg: float
    ) -> Tuple[float, float]:
        """Calculate the hfe and kfe angles.

        This is done given a desired transformed x and z coordinate using the cosine
        rule. The transformed x and z position are described and explained in
        https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics.

        :param transformed_x:
            The desired x_position of the foot, transformed to make the calculation
            easier
        :param transformed_z:
            The desired z_position of the foot, transformed to make the calculation
            easier
        :param upper_leg:
            The length of the upper leg
        :param lower_leg:
            The length of the lower leg

        :return:
            The hip_fe and knee angle needed to reach the desired x and z position
        """
        foot_line_to_leg = acos(
            (
                upper_leg * upper_leg
                + transformed_x * transformed_x
                + transformed_z * transformed_z
                - lower_leg * lower_leg
            )
            / (
                2
                * upper_leg
                * sqrt(transformed_x * transformed_x + transformed_z * transformed_z)
            )
        )
        normal_to_foot_line = atan(transformed_x / transformed_z)
        hfe = foot_line_to_leg + normal_to_foot_line

        sin_normal_lower_leg_angle = round(
            (transformed_x - upper_leg * sin(hfe)) / lower_leg,
            MID_CALCULATION_PRECISION_DIGITS,
        )
        kfe = hfe - asin(sin_normal_lower_leg_angle)

        return hfe, kfe

    @staticmethod
    def calculate_foot_position(haa: float, hfe: float, kfe: float, side: Side) -> Foot:
        """Calculate the foot position given the relevant angles and lengths.

        :param side:
            The side of the exoskeleton to which the angles belong
        :param haa:
            The angle of the hip_aa joint on the specified side
        :param hfe:
            The angle of the hip_fe joint on the specified side
        :param kfe:
            The angle of the knee joint on the specified side

        :return:
            The location of the foot (ankle) of the specified side
            which corresponds to the given angles
        """
        if side != Side.right and side != Side.left:
            raise SideSpecificationError(side)
        # x is positive in the walking direction, z is in the downward direction,
        # y is directed to the right side, the origin in the middle of the hip
        # structure. The calculations are supported by
        # https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics

        ul, ll, hl, ph, base = get_lengths_robot_for_inverse_kinematics(side)

        haa_to_foot_length = ul * cos(hfe) + ll * cos(hfe - kfe)
        z_position = -sin(haa) * ph + cos(haa) * haa_to_foot_length
        x_position = hl + sin(hfe) * ul + sin(hfe - kfe) * ll

        if side == side.left:
            y_position = -cos(haa) * ph - sin(haa) * haa_to_foot_length - base / 2.0
        else:
            y_position = cos(haa) * ph + sin(haa) * haa_to_foot_length + base / 2.0

        return Foot(
            side, Vector3d(x_position, y_position, z_position), Vector3d(0.0, 0.0, 0.0)
        )

    @staticmethod
    def weighted_average_foot(
        base_foot: Foot, other_foot: Foot, parameter: float
    ) -> Foot:
        """Compute the weighted average of two Foot objects.

        :param base_foot:
            The first foot for averaging.
        :param other_foot:
            The second foot for averaging.
        :param parameter:
            The parameter that determines the weight for the feet.

        :return
            The average foot, has a velocity of None if it cannot be computed.
        """
        if base_foot.foot_side != other_foot.foot_side:
            raise SideSpecificationError(
                other_foot.foot_side,
                f"Expected sides of both base and other foot to be equal but "
                f"were {base_foot.foot_side} and {other_foot.foot_side}.",
            )
        resulting_position = weighted_average_vectors(
            base_foot.position, other_foot.position, parameter
        )
        resulting_velocity = weighted_average_vectors(
            base_foot.velocity, other_foot.velocity, parameter
        )
        return Foot(base_foot.foot_side, resulting_position, resulting_velocity)

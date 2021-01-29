from math import acos, asin, atan, cos, pi, sin, sqrt

import rospy

from march_shared_classes.exceptions.gait_exceptions import SubgaitInterpolationError
from march_shared_classes.exceptions.general_exceptions import SideSpecificationError
from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.utilities.side import Side
from march_shared_classes.utilities.utility_functions import (
    get_joint_names_for_inverse_kinematics,
)
from march_shared_classes.utilities.utility_functions import (
    get_lengths_robot_for_inverse_kinematics,
    weighted_average,
)
from march_shared_classes.utilities.vector_3d import Vector3d

VELOCITY_SCALE_FACTOR = 0.001
JOINT_NAMES_IK = get_joint_names_for_inverse_kinematics()
MID_CALCULATION_PRECISION_DIGITS = 10


class Foot(object):
    """Class for capturing the state (position and possible velocity) of a foot."""

    def __init__(self, foot_side, position, velocity=None):
        """Create a Foot object, position and velocity are both Vector3d objects."""
        self.position = position
        self.velocity = velocity
        if foot_side != Side.left and foot_side != Side.right:
            raise SideSpecificationError(foot_side)
        self.foot_side = Side(foot_side)

    def add_foot_velocity_from_next_state(self, next_state):
        """Adds the foot velocity to the state given a next state for the foot to be in.

        :param: next_state: A Foot object that specifies the foot location 1 / VELOCITY_SCALE_FACTOR second later

        :return: The object with a velocity which is calculated based on the next state
        """
        velocity = (next_state.position - self.position) / VELOCITY_SCALE_FACTOR
        self.velocity = velocity

    @classmethod
    def calculate_next_foot_position(cls, current_state):
        """Calculates the foot position a moment later given the current state.

        :param current_state: A Foot object with a velocity

        :return: A Foot object with the position of the foot 1 / VELOCITY_SCALE_FACTOR second later
        """
        next_position = (
            current_state.position + current_state.velocity * VELOCITY_SCALE_FACTOR
        )
        return cls(current_state.foot_side, next_position)

    @staticmethod
    def get_joint_states_from_foot_state(foot_state, time):
        """Translates between feet_state and a list of setpoints, which correspond with the feet_state.

        :param foot_state: A fully populated Foot object.
        :param time: The time of the Foot state and resulting setpoints.

        :return: A dictionary of setpoints, the foot location and velocity of which corresponds with the feet_state.
        """
        joint_states = Foot.calculate_joint_angles_from_foot_position(
            foot_state.position, foot_state.foot_side, time
        )

        # find the joint angles a moment later using the foot position a moment later
        # use this together with the current joint angles to calculate the joint velocity
        next_position = Foot.calculate_next_foot_position(foot_state)
        next_joint_positions = Foot.calculate_joint_angles_from_foot_position(
            next_position.position,
            next_position.foot_side,
            time + VELOCITY_SCALE_FACTOR,
        )

        for joint in JOINT_NAMES_IK:
            if joint in joint_states and joint in next_joint_positions:
                joint_states[joint].add_joint_velocity_from_next_angle(
                    next_joint_positions[joint]
                )

        return joint_states

    @staticmethod
    def calculate_joint_angles_from_foot_position(foot_position, foot_side, time):
        """Calculates the angles of the joints corresponding to a certain position of the right or left foot.

        More information on the calculations of the haa, hfe and kfe angles, aswell as on the velocity calculations can
        be found at https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics. This function
        assumes that the desired z position of the foot is positive.

        :param foot_position: A Vecor3d object which specifies the desired position of the foot.
        :param foot_side: A string which specifies to which side the foot_position belongs and thus which joint angles
            should be computed.
        :param time: The time of the foot_position and the resulting setpoints.

        :return:
            A dictionary of Setpoints for each joint on the requested side with the correct angle at the provided time.
        """
        # Get relevant lengths from robot model, ul = upper leg etc. see get_lengths_robot_for_inverse_kinematics()
        # and unpack desired position
        [ul, ll, hl, ph, base] = get_lengths_robot_for_inverse_kinematics(foot_side)
        x_position = foot_position.x
        y_position = foot_position.y
        z_position = foot_position.z

        # Change y positive direction to the desired foot, change origin to pivot of haa joint, for ease of calculation.
        if foot_side == Side.left:
            y_position = -(y_position + base / 2.0)
        elif foot_side == Side.right:
            y_position = y_position - base / 2.0
        else:
            raise SideSpecificationError(foot_side)

        # Check if the HAA can be calculated (if the desired position is not too close to the origin)
        if ph / sqrt(z_position * z_position + y_position * y_position) > 1:
            raise SubgaitInterpolationError(
                "The desired {foot} foot position, ({x}, {y}, {z}), is out of reach".format(
                    foot=foot_side, x=x_position, y=y_position, z=z_position
                )
            )
        # Then calculate the haa angle. This calculation assumes that pos_z > 0
        haa = Foot.calculate_haa_angle(z_position, y_position, ph)

        # Transform the desired x and z position to arrive at an easier system to calculate
        # the hfe and kfe angles
        transformed_x = round(x_position - hl, MID_CALCULATION_PRECISION_DIGITS)
        transformed_z = round(
            sqrt(-ph * ph + y_position * y_position + z_position * z_position),
            MID_CALCULATION_PRECISION_DIGITS,
        )

        if transformed_x * transformed_x + transformed_z * transformed_z > (ll + ul) * (
            ll + ul
        ):
            raise SubgaitInterpolationError(
                "The desired {foot} foot position, ({x}, {y}, {z}), is out of reach".format(
                    foot=foot_side, x=x_position, y=y_position, z=z_position
                )
            )

        hfe, kfe = Foot.calculate_hfe_kfe_angles(transformed_x, transformed_z, ul, ll)

        # angle positions
        return {
            foot_side.value + "_hip_aa": Setpoint(time, haa),
            foot_side.value + "_hip_fe": Setpoint(time, hfe),
            foot_side.value + "_knee": Setpoint(time, kfe),
        }

    @staticmethod
    def calculate_haa_angle(z_position, y_position, pelvis_hip_length):
        """Calculates the haa angle of the exoskeleton given a desired y and z position of the exoskeleton.

        More information on the calculation is found at
        https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics

        :param z_position: the desired z-position of the foot
        :param y_position: the desired y-position of the foot
        :param pelvis_hip_length: The length from the pelvis to the hip_aa, which is the haa arm

        :return: The hip_aa joint angle needed for the foot to reach the given positions
        """
        if z_position <= 0:
            raise SubgaitInterpolationError(
                "desired z position of the foot is not positive, "
                "current haa calculation is not capable to deal with this"
            )
        haa_arm_to_z_y_distance_ration = pelvis_hip_length / sqrt(
            z_position * z_position + y_position * y_position
        )

        if y_position != 0:
            slope_foot_to_origin = z_position / y_position
            angle_foot_to_origin = atan(slope_foot_to_origin)
            if y_position > 0:
                haa = acos(haa_arm_to_z_y_distance_ration) - angle_foot_to_origin
            else:
                haa = acos(haa_arm_to_z_y_distance_ration) - pi - angle_foot_to_origin
        else:
            angle_foot_to_origin = pi / 2
            haa = acos(haa_arm_to_z_y_distance_ration) - angle_foot_to_origin

        return haa

    @staticmethod
    def calculate_hfe_kfe_angles(transformed_x, transformed_z, upper_leg, lower_leg):
        """Calculates the hfe and kfe given a desired transformed x and z coordinate using the cosine rule.

        The transformed x and z position are described and explained in
        https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics

        :param transformed_x: The desired x_position of the foot, transformed to make the calculation easier
        :param transformed_z: The desired z_position of the foot, transformed to make the calculation easier
        :param upper_leg: The length of the upper leg
        :param lower_leg: The length of the lower leg

        :return: The hip_fe and knee angle needed to reach the desired x and z position
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
    def calculate_foot_position(haa, hfe, kfe, side):
        """Calculates the foot position given the relevant angles, lengths and a specification of the foot.

        :param side: The side of the exoskeleton to which the angles belong
        :param haa: The angle of the hip_aa joint on the specified side
        :param hfe: The angle of the hip_fe joint on the specified side
        :param kfe: The angle of the knee joint on the specified side

        :return: The location of the foot (ankle) of the specified side which corresponds to the given angles
        """
        # x is positive in the walking direction, z is in the downward direction, y is directed to the right side
        # the origin in the middle of the hip structure. The calculations are supported by
        # https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
        ul, ll, hl, ph, base = get_lengths_robot_for_inverse_kinematics(side)
        haa_to_foot_length = ul * cos(hfe) + ll * cos(hfe - kfe)
        z_position = -sin(haa) * ph + cos(haa) * haa_to_foot_length
        x_position = hl + sin(hfe) * ul + sin(hfe - kfe) * ll

        if side == side.left:
            y_position = -cos(haa) * ph - sin(haa) * haa_to_foot_length - base / 2.0
        elif side == side.right:
            y_position = cos(haa) * ph + sin(haa) * haa_to_foot_length + base / 2.0
        else:
            raise SideSpecificationError(side)

        return Foot(side, Vector3d(x_position, y_position, z_position))

    @staticmethod
    def weighted_average_foot(base_foot, other_foot, parameter):
        """Computes the weighted average of two Foot objects, result has a velocity of None if it cannot be computed."""
        if base_foot.foot_side != other_foot.foot_side:
            raise SideSpecificationError(
                "expected sides of both base and other foot to be equal but were {base} and "
                "{other}.".format(base=base_foot.foot_side, other=other_foot.foot_side)
            )
        resulting_position = weighted_average(
            base_foot.position, other_foot.position, parameter
        )
        if base_foot.velocity is not None and other_foot.velocity is not None:
            resulting_velocity = weighted_average(
                base_foot.velocity, other_foot.velocity, parameter
            )
        else:
            rospy.logwarn(
                "one or both of the provided feet does not have a velocity specified, "
                "setting the resulting velocity to None"
            )
            resulting_velocity = None

        return Foot(base_foot.foot_side, resulting_position, resulting_velocity)

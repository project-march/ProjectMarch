from math import acos, atan, cos, pi, sqrt


from march_shared_classes.exceptions.gait_exceptions import SideSpecificationError


from .setpoint import Setpoint
from .utilities import merge_dictionaries, get_lengths_robot_for_inverse_kinematics


VELOCITY_SCALE_FACTOR = 500


class Foot(object):
    """Class for capturing the state (position and possible velocity) of a foot."""

    def __init__(self, foot_side, position, velocity=None):
        """Create a Foot object, position and velocity are both Vector3d objects."""
        self.position = position
        self.velocity = velocity
        self.foot_side = foot_side
        if foot_side != 'left' and foot_side != 'right':
            raise SideSpecificationError(foot_side)

    def add_foot_velocity_from_next_state(self, next_state):
        """Adds the foot velocity to the state given a next state for the foot to be in.

        :param: next_state: A Foot object that specifies the foot location 1 / VELOCITY_SCALE_FACTOR second later

        :return: The object with a velocity which is calculated based on the next state
        """
        velocity = (next_state.position - self.position) * VELOCITY_SCALE_FACTOR
        self.velocity = velocity

    @classmethod
    def calculate_next_foot_position(cls, current_state):
        """Calculates the foot position a moment later given the current state.

        :param current_state: A Foot object with a velocity

        :return A Foot object with the position of the foot 1 / VELOCITY_SCALE_FACTOR second later
        """
        next_position = current_state.position + current_state.velocity / VELOCITY_SCALE_FACTOR
        return cls(current_state.foot_side, next_position)

    @staticmethod
    def get_joint_states_from_foot_state(foot_state):
        """Computes the state (position & velocity) of the joints needed to reach a desired state of the foot.

        :param foot_state:
            A Foot object which specifies the desired position and velocity of the foot, also specifies which foot.

        :return:
            A dictionary with an entry for each joint on the requested side with the correct joint angles and joint
            velocities.
        """
        # find the joint angles a moment later using the foot position a moment later
        # use this together with the current joint angles to calculate the joint velocity
        angle_positions = Setpoint.calculate_joint_angles_from_foot_position(foot_state.position,
                                                                             foot_state.foot_side)

        next_position = Foot.calculate_next_foot_position(foot_state)

        next_angles = Setpoint.calculate_joint_angles_from_foot_position(next_position.position,
                                                                         next_position.foot_side)

        angle_velocities = Setpoint.calculate_joint_velocities(next_angles, angle_positions, foot_state.foot_side)

        angle_states = merge_dictionaries(angle_velocities, angle_positions)

        return angle_states

    @staticmethod
    def calculate_joint_angles_from_foot_position(foot_position, foot_side):
        """Calculates the angles of the joints corresponding to a certain position of the right or left foot.

        More information on the calculations of the haa, hfe and kfe angles, aswell as on the velocity calculations can
        be found at https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics. This function
        assumes that the desired z position of the foot is positive.

        :param foot_position:
            A Vecor3d object which specifies the desired position of the foot.
        :param foot_side:
            A string which specifies to which side the foot_position belongs and thus which joint angles should be
            computed.

        :return:
            A dictionary with an entry for each joint on the requested side with the correct angle.
        """
        # Get relevant lengths from robot model, ul = upper leg etc. see get_lengths_robot_for_inverse_kinematics()
        # and unpack desired position
        [ul, ll, hl, ph, base] = get_lengths_robot_for_inverse_kinematics(foot_side)
        x_position = foot_position.x
        y_position = foot_position.y
        z_position = foot_position.z

        # Change y positive direction to the desired foot, change origin to pivot of haa joint, for ease of calculation.
        if foot_side == 'left':
            y_position = - (y_position + base / 2.0)
        elif foot_side == 'right':
            y_position = y_position - base / 2.0
        else:
            raise SideSpecificationError(foot_side)

        # first calculate the haa angle. This calculation assumes that pos_z > 0
        haa = Setpoint.calculate_haa_angle(z_position, y_position, ph)

        # once the haa angle is known, rescale the desired x and z position to arrive at an easier system to calculate
        # the hfe and kfe angles
        rescaled_x = round(x_position - hl, 10)
        rescaled_z = round(sqrt(- ph * ph + y_position * y_position + z_position * z_position), 10)

        if rescaled_x * rescaled_x + rescaled_z * rescaled_z > (ll + ul) * (ll + ul):
            raise SubgaitInterpolationError('The desired {foot} foot position, ({x}, {y}, {z}), is out of reach'.
                                            format(foot=foot_side, x=x_position, y=y_position, z=z_position))

        hfe, kfe = Setpoint.calculate_hfe_kfe_angles(rescaled_x, rescaled_z, ul, ll)

        angle_positions = {foot_side + '_hip_aa': haa, foot_side + '_hip_fe': hfe, foot_side + '_knee': kfe}
        return angle_positions


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
                'desired z position of the foot is not positive, current haa calculation is not capable of this')

        if y_position != 0:
            slope_foot_to_origin = z_position / y_position
            angle_foot_to_origin = atan(slope_foot_to_origin)
            if y_position > 0:
                haa = acos(pelvis_hip_length / sqrt(z_position * z_position + y_position * y_position)) \
                    - angle_foot_to_origin
            else:
                haa = acos(pelvis_hip_length / sqrt(z_position * z_position + y_position * y_position)) \
                    - pi - angle_foot_to_origin
        else:
            angle_foot_to_origin = pi / 2
            haa = acos(pelvis_hip_length / sqrt(z_position * z_position + y_position * y_position)) \
                - angle_foot_to_origin

        return haa

    @staticmethod
    def calculate_hfe_kfe_angles(rescaled_x, rescaled_z, upper_leg, lower_leg):
        """Calculates the hfe and kfe given a desired rescaled x and z coordinate using the cosine rule.

        The rescaled x and z position are described and explained in
        https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics

        :param rescaled_x: The desired x_position of the foot, rescaled to make the calculation easier
        :param rescaled_z: The desired z_position of the foot, rescaled to make the calculation easier
        :param upper_leg: The length of the upper leg
        :param lower_leg: The length of the lower leg

        :return: The hip_fe and knee angle needed to reach the desired x and z position
        """
        foot_line_to_leg = acos((upper_leg * upper_leg + rescaled_x * rescaled_x + rescaled_z * rescaled_z
                                 - lower_leg * lower_leg)
                                / (2 * upper_leg * sqrt(rescaled_x * rescaled_x + rescaled_z * rescaled_z)))
        normal_to_foot_line = atan(rescaled_x / rescaled_z)
        hfe = foot_line_to_leg + normal_to_foot_line
        kfe = acos((rescaled_z - upper_leg * cos(hfe)) / lower_leg) + hfe

        return hfe, kfe

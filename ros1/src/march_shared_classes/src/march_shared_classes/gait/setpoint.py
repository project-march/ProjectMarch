from math import acos, atan, cos, pi, sin, sqrt
import os

import rospkg
from urdf_parser_py import urdf

from march_shared_classes.exceptions.gait_exceptions import SideSpecificationError, SubgaitInterpolationError

from .feet_state import FeetState
from .foot import Foot
from .utilities import merge_dictionaries, weighted_average
from .vector_3d import Vector3d

# Use this factor when calculating velocities to keep the calculations within the range of motion
# See IK confluence page https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
VELOCITY_SCALE_FACTOR = 500
JOINTS_POSITION_NAMES_IK = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'right_hip_aa', 'right_hip_fe', 'right_knee']
JOINTS_VELOCITY_NAMES_IK = ['left_hip_aa_velocity', 'left_hip_fe_velocity', 'left_knee_velocity',
                            'right_hip_aa_velocity', 'right_hip_fe_velocity', 'right_knee_velocity']


class Setpoint(object):
    """Base class to define the setpoints of a subgait."""

    digits = 4

    def __init__(self, time, position, velocity):
        self._time = round(time, self.digits)
        self._position = round(position, self.digits)
        self._velocity = round(velocity, self.digits)

    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, time):
        self._time = round(time, self.digits)

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position = round(position, self.digits)

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, velocity):
        self._velocity = round(velocity, self.digits)

    @staticmethod
    def create_position_interpolated_setpoints(base_setpoints, other_setpoints, parameter):
        """Linearly interpolate between the foot position.

        :param base_setpoints:
            A dictionary of setpoints, one for each joint. Return value if parameter is zero
        :param other_setpoints:
            A dictionary of setpoints, one for each joint. Return value if parameter is one
        :param parameter:
            parameter for the interpolation
        :return
            A dictionary of setpoints, who's corresponding foot location is linearly interpolated from the setpoints
        """
        base_foot_state = Setpoint.get_feet_state_from_setpoints(base_setpoints)
        other_foot_state = Setpoint.get_feet_state_from_setpoints(other_setpoints)

        new_feet_state = FeetState.weighted_average_states(base_foot_state, other_foot_state, parameter)

        new_joint_states = merge_dictionaries(
            Setpoint.get_joint_states_from_foot_state(new_feet_state.left_foot),
            Setpoint.get_joint_states_from_foot_state(new_feet_state.right_foot))

        # linearly interpolate the ankle angle, as it cannot be calculated from the inverse kinematics
        try:
            new_ankle_angle_left = weighted_average(base_setpoints['left_ankle'].position,
                                                    other_setpoints['left_ankle'].position, parameter)
            new_ankle_angle_right = weighted_average(base_setpoints['right_ankle'].position,
                                                     other_setpoints['right_ankle'].position, parameter)
            new_ankle_velocity_left = weighted_average(base_setpoints['left_ankle'].velocity,
                                                       other_setpoints['left_ankle'].velocity, parameter)
            new_ankle_velocity_right = weighted_average(base_setpoints['right_ankle'].velocity,
                                                        other_setpoints['right_ankle'].velocity, parameter)
        except KeyError as e:
            raise KeyError('Expected setpoint dictionaries to contain "{key}", but "{key}" was missing.'.
                           format(key=e.args[0]))

        # Set the time of the new setpoints as the weighted average of the original setpoint times
        base_setpoints_time = 0
        other_setpoints_time = 0
        for setpoint in base_setpoints.values():
            base_setpoints_time += setpoint.time
        for setpoint in other_setpoints.values():
            other_setpoints_time += setpoint.time
        time = weighted_average(base_setpoints_time, other_setpoints_time, parameter) / len(base_setpoints)

        resulting_setpoints = {'left_ankle': Setpoint(time, new_ankle_angle_left, new_ankle_velocity_left),
                               'right_ankle': Setpoint(time, new_ankle_angle_right, new_ankle_velocity_right)}
        for joint, velocity in zip(JOINTS_POSITION_NAMES_IK, JOINTS_VELOCITY_NAMES_IK):
            resulting_setpoints[joint] = Setpoint(time, new_joint_states[joint], new_joint_states[velocity])

        return resulting_setpoints

    def __repr__(self):
        return 'Time: %s, Position: %s, Velocity: %s' % (self.time, self.position, self.velocity)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return (self.time == other.time
                    and self.position == other.position
                    and self.velocity == other.velocity)
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

    @staticmethod
    def interpolate_setpoints(base_setpoint, other_setpoint, parameter):
        """Linearly interpolate two setpoints.

        :param base_setpoint:
            base setpoint, return value if parameter is zero
        :param other_setpoint:
            other setpoint, return value if parameter is one
        :param parameter:
            parameter for linear interpolation, 0 <= parameter <= 1
        :return:
            The interpolated setpoint
        """
        time = weighted_average(base_setpoint.time, other_setpoint.time, parameter)
        position = weighted_average(base_setpoint.position, other_setpoint.position, parameter)
        velocity = weighted_average(base_setpoint.velocity, other_setpoint.velocity, parameter)
        return Setpoint(time, position, velocity)

    @staticmethod
    def get_feet_state_from_setpoints(setpoint_dic):
        """Calculate the position and velocity of the foot (or rather ankle) from joint angles.

        :param setpoint_dic:
            Dictionary of setpoints from which the feet positions and velocities need to be calculated

        :return:
            A FeetState object with a left and right foot which each have a position and velocity corresponding to the
            setpoint dictionary
        """
        for joint in JOINTS_POSITION_NAMES_IK:
            if joint not in setpoint_dic:
                raise KeyError('expected setpoint dictionary to contain joint {joint}, but {joint} was missing.'.
                               format(joint=joint))

        foot_state_left = Setpoint.calculate_foot_position(setpoint_dic['left_hip_aa'].position,
                                                           setpoint_dic['left_hip_fe'].position,
                                                           setpoint_dic['left_knee'].position, 'left')
        foot_state_right = Setpoint.calculate_foot_position(setpoint_dic['right_hip_aa'].position,
                                                            setpoint_dic['right_hip_fe'].position,
                                                            setpoint_dic['right_knee'].position, 'right')

        next_joint_positions = Setpoint.calculate_next_positions_joint(setpoint_dic)

        next_foot_state_left = Setpoint.calculate_foot_position(next_joint_positions['left_hip_aa'],
                                                                next_joint_positions['left_hip_fe'],
                                                                next_joint_positions['left_knee'], 'left')
        next_foot_state_right = Setpoint.calculate_foot_position(next_joint_positions['right_hip_aa'],
                                                                 next_joint_positions['right_hip_fe'],
                                                                 next_joint_positions['right_knee'], 'right')

        foot_state_left.add_foot_velocity_from_next_state(next_foot_state_left)
        foot_state_right.add_foot_velocity_from_next_state(next_foot_state_right)

        feet_state = FeetState(foot_state_right, foot_state_left)
        return feet_state

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
        ul, ll, hl, ph, base = Setpoint.get_lengths_robot_for_inverse_kinematics(side)
        haa_to_foot_length = ul * cos(hfe) + ll * cos(hfe - kfe)
        z_position = - sin(haa) * ph + cos(haa) * haa_to_foot_length
        x_position = hl + sin(hfe) * ul + sin(hfe - kfe) * ll

        if side == 'left':
            y_position = - cos(haa) * ph - sin(haa) * haa_to_foot_length - base / 2.0
        elif side == 'right':
            y_position = cos(haa) * ph + sin(haa) * haa_to_foot_length + base / 2.0
        else:
            raise SideSpecificationError(side)

        return Foot(side, Vector3d(x_position, y_position, z_position))

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
        [ul, ll, hl, ph, base] = Setpoint.get_lengths_robot_for_inverse_kinematics(foot_side)
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
    def get_lengths_robot_for_inverse_kinematics(side=None):
        """Grabs lengths from the robot which are relevant for the inverse kinematics calculation.

        this function returns the lengths relevant for the specified foot, if no side is specified,
        it returns all relevant lengths for both feet.

        :param side: The side of the exoskeleton of which the relevant would like to be known
        :return: The lengths of the specified side which are relevant for the (inverse) kinematics calculations
        """
        try:
            robot = urdf.Robot.from_xml_file(os.path.join(rospkg.RosPack().get_path('march_description'), 'urdf',
                                                          'march4.urdf'))
            # size[0], size[1] and size[2] are used to grab relevant length of the link, e.g. the relevant length of the
            # hip base is in the y direction, that of the upper leg in the z direction.
            base = robot.link_map['hip_base'].collisions[0].geometry.size[1]  # length of the hip base structure
            l_ul = robot.link_map['upper_leg_left'].collisions[0].geometry.size[2]  # left upper leg length
            l_ll = robot.link_map['lower_leg_left'].collisions[0].geometry.size[2]  # left lower leg length
            l_hl = robot.link_map['hip_aa_frame_left_front'].collisions[0].geometry.size[0]  # left haa arm to leg
            l_ph = robot.link_map['hip_aa_frame_left_side'].collisions[0].geometry.size[1]  # left pelvis to hip length
            r_ul = robot.link_map['upper_leg_right'].collisions[0].geometry.size[2]  # right upper leg length
            r_ll = robot.link_map['lower_leg_right'].collisions[0].geometry.size[2]  # right lower leg length
            r_hl = robot.link_map['hip_aa_frame_right_front'].collisions[0].geometry.size[0]  # right haa arm to leg
            r_ph = robot.link_map['hip_aa_frame_right_side'].collisions[0].geometry.size[1]  # right pelvis hip length
            # the foot is a certain amount more to the inside of the exo then the leg structures.
            # The haa arms need to account for this.
            off_set = robot.link_map['ankle_plate_right'].visual.origin.xyz[1] + base / 2 + r_hl
            r_hl = r_hl - off_set
            l_hl - l_hl - off_set

        except KeyError as e:
            raise KeyError('Expected robot.link_map to contain "{key}", but "{key}" was missing.'.
                           format(key=e.args[0]))

        if side == 'left':
            return [l_ul, l_ll, l_hl, l_ph, base]
        elif side == 'right':
            return [r_ul, r_ll, r_hl, r_ph, base]
        elif side == 'both':
            return [l_ul, l_ll, l_hl, l_ph, r_ul, r_ll, r_hl, r_ph, base]
        else:
            raise SideSpecificationError(side, "Side should be either 'left', 'right' or 'both', but was {side}".
                                         format(side=side))

    @staticmethod
    def calculate_next_positions_joint(setpoint_dic):
        """Calculates the position of the joints a moment later given a setpoint dictionary.

        :param setpoint_dic: A dictionary of setpoints with positions and velocities
        :return: A dictionary with the positions of the joints 1 / VELOCITY_SCALE_FACTOR second later
        """
        next_positions = {}
        for joint in JOINTS_POSITION_NAMES_IK:
            if joint in setpoint_dic:
                next_positions[joint] = setpoint_dic[joint].position + setpoint_dic[joint].velocity \
                    / VELOCITY_SCALE_FACTOR
            else:
                raise KeyError('setpoint_dic is missing joint {joint}'.format(joint=joint))

        return next_positions

    @staticmethod
    def calculate_joint_velocities(next_angle_positions, angle_positions, side='both'):
        """Calculates the (left/right/all)joint velocities given a current position and a next position.

        :param next_angle_positions: A dictionary containing the positions of the joints a moment later
        :param angle_positions: A dictionary containing the current positions of the joints
        :param side: Specifies the side of the exo whos joints should be used for the calculation

        ":return: The joint velocities of the joints on the specified side
        """
        if side != 'left' and side != 'right' and side != 'both':
            raise SideSpecificationError(side, "Side should be either 'left', 'right' or 'both', but was {side}".
                                         format(side=side))
        angle_velocity = {}
        for coordinate, velocity in zip(JOINTS_POSITION_NAMES_IK, JOINTS_VELOCITY_NAMES_IK):
            if side == 'left' or side == 'right':
                if not (coordinate.startswith(side) and velocity.startswith(side)):
                    continue
            if coordinate in next_angle_positions and coordinate in angle_positions:
                angle_velocity[velocity] = (next_angle_positions[coordinate] - angle_positions[coordinate]) \
                    * VELOCITY_SCALE_FACTOR
            else:
                raise KeyError('next_angle_positions or angle_positions is missing key {coord}'.
                               format(coord=coordinate))

        return angle_velocity

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

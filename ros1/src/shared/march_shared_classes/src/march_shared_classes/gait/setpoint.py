from math import acos, atan, cos, pi, sin, sqrt

import rospkg
from urdf_parser_py import urdf
import rospy

from march_shared_classes.exceptions.gait_exceptions import SideSpecificationError, SubgaitInterpolationError

# Use this factor when calculating velocities to keep the calculations within the range of motion
# See IK confluence page https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
VELOCITY_SCALE_FACTOR = 500
JOINTS_POSITION_NAMES = ['left_hip_aa', 'left_hip_fe', 'left_knee', 'right_hip_aa', 'right_hip_fe', 'right_knee']
JOINTS_VELOCITY_NAMES = ['left_hip_aa_velocity', 'left_hip_fe_velocity', 'left_knee_velocity',
                         'right_hip_aa_velocity', 'right_hip_fe_velocity', 'right_knee_velocity']
FOOT_COORDINATE_NAMES = ['left_foot_x', 'left_foot_y', 'left_foot_z', 'right_foot_x', 'right_foot_y', 'right_foot_z']
FOOT_VELOCITY_NAMES = ['left_foot_x_velocity', 'left_foot_y_velocity', 'left_foot_z_velocity',
                       'right_foot_x_velocity', 'right_foot_y_velocity', 'right_foot_z_velocity']


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
        base_foot_position = Setpoint.get_foot_pos_from_angles(base_setpoints)
        base_foot_velocity = Setpoint.get_foot_pos_from_angles(base_setpoints, velocity=True)
        other_foot_position = Setpoint.get_foot_pos_from_angles(other_setpoints)
        other_foot_velocity = Setpoint.get_foot_pos_from_angles(other_setpoints, velocity=True)

        new_foot_pos = Setpoint.weighted_average_dictionary(base_foot_position, other_foot_position, parameter)
        new_foot_vel = Setpoint.weighted_average_dictionary(base_foot_velocity, other_foot_velocity, parameter)

        new_angles_left = Setpoint.calculate_joint_angles_from_foot_position(new_foot_pos, 'left',
                                                                             foot_velocity=new_foot_vel)
        new_angles_right = Setpoint.calculate_joint_angles_from_foot_position(new_foot_pos, 'right',
                                                                              foot_velocity=new_foot_vel)

        # linearly interpolate the ankle angle, as it cannot be calculated from the inverse kinematics
        try:
            new_ankle_angle_left = Setpoint.weighted_average(base_setpoints['left_ankle'].position,
                                                             other_setpoints['left_ankle'].position, parameter)
            new_ankle_angle_right = Setpoint.weighted_average(base_setpoints['right_ankle'].position,
                                                              other_setpoints['right_ankle'].position, parameter)
            new_ankle_velocity_left = Setpoint.weighted_average(base_setpoints['left_ankle'].velocity,
                                                                other_setpoints['left_ankle'].velocity, parameter)
            new_ankle_velocity_right = Setpoint.weighted_average(base_setpoints['right_ankle'].velocity,
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
        time = Setpoint.weighted_average(base_setpoints_time, other_setpoints_time, parameter) / len(base_setpoints)

        resulting_setpoints = {'left_hip_aa': Setpoint(time, new_angles_left['left_hip_aa'],
                                                       new_angles_left['left_hip_aa_velocity']),
                               'left_hip_fe': Setpoint(time, new_angles_left['left_hip_fe'],
                                                       new_angles_left['left_hip_fe_velocity']),
                               'left_knee': Setpoint(time, new_angles_left['left_knee'],
                                                     new_angles_left['left_knee_velocity']),
                               'right_hip_aa': Setpoint(time, new_angles_right['right_hip_aa'],
                                                        new_angles_right['right_hip_aa_velocity']),
                               'right_hip_fe': Setpoint(time, new_angles_right['right_hip_fe'],
                                                        new_angles_right['right_hip_fe_velocity']),
                               'right_knee': Setpoint(time, new_angles_right['right_knee'],
                                                      new_angles_right['right_knee_velocity']),
                               'left_ankle': Setpoint(time, new_ankle_angle_left, new_ankle_velocity_left),
                               'right_ankle': Setpoint(time, new_ankle_angle_right, new_ankle_velocity_right)}

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
        time = Setpoint.weighted_average(base_setpoint.time, other_setpoint.time, parameter)
        position = Setpoint.weighted_average(base_setpoint.position, other_setpoint.position, parameter)
        velocity = Setpoint.weighted_average(base_setpoint.velocity, other_setpoint.velocity, parameter)
        return Setpoint(time, position, velocity)

    @staticmethod
    def get_foot_pos_from_angles(setpoint_dic, velocity=False):
        """Calculate the position of the foot (ankle, ADFP, is not taken into account) from joint angles.

        :param setpoint_dic:
            Dictionary of setpoints from which the foot positions need to be calculated
        :param velocity:
            Boolean which determines whether the foot position or the foot velocity needs to be calculated

        :return:
            the foot location or velocity as a dictionary. Origin in the hip base, y positive to the right.
        """
        for joint in JOINTS_POSITION_NAMES:
            if joint not in setpoint_dic:
                raise KeyError('expected setpoint dictionary to contain joint {joint}, but {joint} was missing.'.
                               format(joint=joint))

        foot_positions = Setpoint.merge_dictionaries(
            Setpoint.calculate_foot_position(setpoint_dic['left_hip_aa'].position,
                                             setpoint_dic['left_hip_fe'].position,
                                             setpoint_dic['left_knee'].position, 'left'),
            Setpoint.calculate_foot_position(setpoint_dic['right_hip_aa'].position,
                                             setpoint_dic['right_hip_fe'].position,
                                             setpoint_dic['right_knee'].position, 'right'))

        if velocity:
            # To calculate the velocity of the foot, find the foot location as it would be a fraction of a second later.
            next_joint_positions = Setpoint.calculate_next_positions_joint(setpoint_dic)

            next_foot_positions = Setpoint.merge_dictionaries(
                Setpoint.calculate_foot_position(next_joint_positions['left_hip_aa'],
                                                 next_joint_positions['left_hip_fe'],
                                                 next_joint_positions['left_knee'], 'left'),
                Setpoint.calculate_foot_position(next_joint_positions['right_hip_aa'],
                                                 next_joint_positions['right_hip_fe'],
                                                 next_joint_positions['right_knee'], 'right'))

            foot_velocity = Setpoint.calculate_foot_velocity(next_foot_positions, foot_positions)

            return foot_velocity

        else:
            return foot_positions

    @staticmethod
    def calculate_foot_position(haa, hfe, kfe, side):
        """Calculates the foot position given the relevant angles, lengths and a specification of the foot."""
        # x is positive in the walking direction, z is in the downward direction, y is directed to the right side
        # the origin in the middle of the hip structure. The calculations are supported by
        # https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
        ul, ll, hl, ph, base = Setpoint.get_lengths_robot(side)
        haa_to_foot_length = ul * cos(hfe) + ll * cos(hfe - kfe)
        z_position = - sin(haa) * ph + cos(haa) * haa_to_foot_length
        x_position = hl + sin(hfe) * ul + sin(hfe - kfe) * ll
        if side == 'left':
            y_position = - cos(haa) * ph - sin(haa) * haa_to_foot_length - base / 2.0
            return {'left_foot_x': x_position, 'left_foot_y': y_position, 'left_foot_z': z_position}
        elif side == 'right':
            y_position = cos(haa) * ph + sin(haa) * haa_to_foot_length + base / 2.0
            return {'right_foot_x': x_position, 'right_foot_y': y_position, 'right_foot_z': z_position}
        else:
            raise SideSpecificationError(side)


    @staticmethod
    def calculate_joint_angles_from_foot_position(foot_position, foot, foot_velocity=None):
        """Calculates the angles of the joints corresponding to a certain position of the right or left foot.

        More information on the calculations of the haa, hfe and kfe angles, aswel as on the velocity calculations can
        be found at https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics. This function
        assumes that the desired z position of the foot is positive.

        :param position:
            Dictionary that specifies the x, y and z position of the foot. Origin in the hip base,
            y positive to the right
        :param foot:
            String that specifies to which foot the coordinates in position belong
        :param velocity:
            Dictionary that specifies the velocity vector of the foot

        :return:
            A dictionary with an entry for each joint on the requested side with the correct angle when velocity is not
            requested. When it is the dictionary also has an entry for the velocity of each joint.
        """
        # Change y positive direction to the desired foot, change origin to pivot of haa joint, for ease of calculation.
        # Get relevant lengths from robot model, ul = upper leg etc. see get_lengths_robot().
        # If a velocity is supplied, also find the position the foot would be in a moment later,
        # without scaling this could be invalid
        if foot == 'left':
            pos_x = foot_position['left_foot_x']
            pos_z = foot_position['left_foot_z']
            [ul, ll, hl, ph, base] = Setpoint.get_lengths_robot('left')
            pos_y = - (foot_position['left_foot_y'] + base / 2.0)
        elif foot == 'right':
            pos_x = foot_position['right_foot_x']
            pos_z = foot_position['right_foot_z']
            [ul, ll, hl, ph, base] = Setpoint.get_lengths_robot('right')
            pos_y = foot_position['right_foot_y'] - base / 2.0
        else:
            raise SideSpecificationError(foot)

        # first calculate the haa angle. This calculation assumes that pos_z > 0
        haa = Setpoint.calculate_haa_angle(pos_z, pos_y, ph)

        # once the haa angle is known, rescale the desired x and z position to arrive at easier system to calculate the
        # hfe and kfe angles
        rescaled_x = round(pos_x - hl, 10)
        rescaled_z = round(sqrt(- ph * ph + pos_y * pos_y + pos_z * pos_z), 10)

        if rescaled_x * rescaled_x + rescaled_z * rescaled_z > (ll + ul) * (ll + ul):
            raise SubgaitInterpolationError('The desired foot position, ({0}, {1}, {2}), is out of reach'.
                                            format(pos_x, pos_y, pos_z))

        hfe, kfe = Setpoint.calculate_hfe_kfe_angles(rescaled_x, rescaled_z, ul, ll)

        angle_positions = {foot + '_hip_aa': haa, foot + '_hip_fe': hfe, foot + '_knee': kfe}

        if bool(foot_velocity):
            # find the angles a moment later using the foot position a moment later
            # scale the found velocity back to radians per second
            next_position = Setpoint.calculate_next_position_foot(foot_position, foot_velocity)
            next_angles = Setpoint.calculate_joint_angles_from_foot_position(next_position, foot)
            angle_velocities = Setpoint.calculate_joint_velocities(next_angles, angle_positions)
            angle_velocities.update(angle_positions)
            return angle_velocities
        else:
            return angle_positions

    @staticmethod
    def weighted_average_dictionary(base_dictionary, other_dictionary, parameter):
        """Computes the weighted average of the entries of two dictionaries with normalised weight parameter."""
        if len(base_dictionary) != len(other_dictionary):
            raise KeyError('Dictionaries do not have the same number of entries.')
        resulting_dictionary = {}
        for key in base_dictionary.keys():
            try:
                resulting_dictionary[key] = Setpoint.weighted_average(base_dictionary[key],
                                                                      other_dictionary[key], parameter)
            except KeyError as e:
                raise KeyError('Dictionaries must have the same keys for a weighted average. other_dictionary misses '
                               '{key}'.format(key=e.args[0]))
        return resulting_dictionary

    @staticmethod
    def weighted_average(base_value, other_value, parameter):
        """Compute the weighted average of two values with normalised weight parameter."""
        return base_value * (1 - parameter) + other_value * parameter

    @staticmethod
    def get_lengths_robot(side=None):
        """Grabs lengths from the robot which are relevant for the inverse kinematics calculation.

        this function returns the lengths relevant for the specified foot, if no side is specified,
        it returns all relevant lengths for both feet.
        """
        try:
            robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')
            l_ul = robot.link_map['upper_leg_left'].collisions[0].geometry.size[2]  # left upper leg length
            l_ll = robot.link_map['lower_leg_left'].collisions[0].geometry.size[2]  # left lower leg length
            l_hl = robot.link_map['hip_aa_frame_left_front'].collisions[0].geometry.size[0]  # left haa arm to leg
            l_ph = robot.link_map['hip_aa_frame_left_side'].collisions[0].geometry.size[1]  # left pelvis to hip length
            r_ul = robot.link_map['upper_leg_right'].collisions[0].geometry.size[2]  # right upper leg length
            r_ll = robot.link_map['lower_leg_right'].collisions[0].geometry.size[2]  # right lower leg length
            r_hl = robot.link_map['hip_aa_frame_right_front'].collisions[0].geometry.size[0]  # right haa arm to leg
            r_ph = robot.link_map['hip_aa_frame_right_side'].collisions[0].geometry.size[1]  # right pelvis hip length
            base = robot.link_map['hip_base'].collisions[0].geometry.size[1]  # length of the hip base structure
        except KeyError as e:
            raise KeyError('Expected robot.link_map to contain "{key}", but "{key}" was missing.'.
                           format(key=e.args[0]))
        if side == 'left':
            return [l_ul, l_ll, l_hl, l_ph, base]
        elif side == 'right':
            return [r_ul, r_ll, r_hl, r_ph, base]
        elif side is None:
            return [l_ul, l_ll, l_hl, l_ph, r_ul, r_ll, r_hl, r_ph, base]
        else:
            raise SideSpecificationError(side)

    @staticmethod
    def merge_dictionaries(dic_one, dic_two):
        merged_dic = {}
        for key_one in dic_one:
            if key_one not in dic_two or dic_one[key_one] == dic_two[key_one]:
                merged_dic[key_one] = dic_one[key_one]
            else:
                raise KeyError('Dictionaries to be merged both contain key {key} with differing values'.
                               format(key=key_one))
        for key_two in dic_two:
            if key_two not in dic_one or dic_one[key_two] == dic_two[key_two]:
                merged_dic[key_two] = dic_two[key_two]
            else:
                raise KeyError('Dictionaries to be merged both contain key {key} with differing values'.
                               format(key=key_two))
        return merged_dic


    @staticmethod
    def calculate_next_positions_joint(setpoint_dic):
        """Calculates the position of the joints a moment (1 / VELOCITY_SCALE_FACTOR second) later."""
        next_positions = {}
        for joint in JOINTS_POSITION_NAMES:
            if joint in setpoint_dic:
                next_positions[joint] = setpoint_dic[joint].position + setpoint_dic[joint].velocity \
                                        / VELOCITY_SCALE_FACTOR
            else:
                raise KeyError('setpoint_dic is missing joint {joint}'.format(joint=joint))
        return next_positions

    @staticmethod
    def calculate_next_position_foot(position_dic, velocity_dic):
        """Calculates the position of the joints a moment (1 / VELOCITY_SCALE_FACTOR second) later."""
        next_position = {}
        for coordinate, velocity in zip(FOOT_COORDINATE_NAMES, FOOT_VELOCITY_NAMES):
            if coordinate in position_dic and velocity in velocity_dic:
                next_position[coordinate] = position_dic[coordinate] + velocity_dic[velocity] \
                                            / VELOCITY_SCALE_FACTOR
            else:
                rospy.logwarn('position_dic or velocity_dic is missing key {coord}'.format(coord=coordinate))
        return next_position

    @staticmethod
    def calculate_foot_velocity(next_foot_positions, foot_positions):
        foot_velocity = {}
        for coordinate, velocity in zip(FOOT_COORDINATE_NAMES, FOOT_VELOCITY_NAMES):
            if coordinate in next_foot_positions and coordinate in foot_positions:
                foot_velocity[velocity] = (next_foot_positions[coordinate] - foot_positions[coordinate]) \
                                            * VELOCITY_SCALE_FACTOR
            else:
                raise KeyError('next_foot_positions or foot_positions is missing key {coord}'.format(coord=coordinate))
        return foot_velocity

    @staticmethod
    def calculate_joint_velocities(next_angle_positions, angle_positions):
        angle_velocity = {}
        for coordinate, velocity in zip(JOINTS_POSITION_NAMES, JOINTS_VELOCITY_NAMES):
            if coordinate in next_angle_positions and coordinate in angle_positions:
                angle_velocity[velocity] = (next_angle_positions[coordinate] - angle_positions[coordinate]) \
                                            * VELOCITY_SCALE_FACTOR
            else:
                rospy.logwarn('next_angle_positions or angle_positions is missing key {coord}'.format(coord=coordinate))
        return angle_velocity

    @staticmethod
    def calculate_haa_angle(pos_z, pos_y, pelvis_hip_length):
        """Calculates the haa angle of the exoskeleton given a desired y and z position of the exoskeleton."""
        if pos_z <= 0:
            raise SubgaitInterpolationError('desired z position of the foot is not positive, current haa calculation is '
                                            'not capable of this')
        if pos_y != 0:
            slope_y_to_or = pos_z / pos_y
            alpha = atan(slope_y_to_or)
            if pos_y > 0:
                haa = acos(pelvis_hip_length / sqrt(pos_z * pos_z + pos_y * pos_y)) - alpha
            else:
                haa = acos(pelvis_hip_length / sqrt(pos_z * pos_z + pos_y * pos_y)) - pi - alpha
        else:
            alpha = pi / 2
            haa = acos(pelvis_hip_length / sqrt(pos_z * pos_z + pos_y * pos_y)) - alpha
        return haa

    @staticmethod
    def calculate_hfe_kfe_angles(rescaled_x, rescaled_z, upper_leg, lower_leg):
        foot_line_to_leg = acos((upper_leg * upper_leg + rescaled_x * rescaled_x + rescaled_z * rescaled_z
                                 - lower_leg * lower_leg)
                                / (2 * upper_leg * sqrt(rescaled_x * rescaled_x + rescaled_z * rescaled_z)))
        normal_to_foot_line = atan(rescaled_x / rescaled_z)
        hfe = foot_line_to_leg + normal_to_foot_line
        kfe = acos((rescaled_z - upper_leg * cos(hfe)) / lower_leg) + hfe
        return hfe, kfe


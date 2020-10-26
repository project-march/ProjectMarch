from math import *
from march_shared_classes.exceptions.gait_exceptions import SubgaitInterpolationError
import numpy as np

import rospkg
from urdf_parser_py import urdf


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

    @classmethod
    def interpolate_setpoints_position(cls, base_setpoints, other_setpoints, parameter):
        """Linearly interpolate the ankle position.

        :param base_setpoints:
            A dictionary of setpoints, one for each joint. Return value if parameter is zero
        :param other_setpoints:
            A dictionary of setpoints, one for each joint. Return value if parameter is one
        :param parameter:
            parameter for the interpolation
        :return
            A dictionary of setpoints, who's corresponding foot location is linearly interpolated from the setpoints"""

        base_foot_pos = np.array(Setpoint.get_foot_pos_from_angles(base_setpoints))
        base_foot_vel = np.array(Setpoint.get_foot_pos_from_angles(base_setpoints, velocity=True))
        other_foot_pos = np.array(Setpoint.get_foot_pos_from_angles(other_setpoints))
        other_foot_vel = np.array(Setpoint.get_foot_pos_from_angles(other_setpoints, velocity=True))

        new_foot_pos = base_foot_pos * (1 - parameter) + other_foot_pos * parameter
        new_foot_vel = (base_foot_vel * (1 - parameter) + other_foot_vel * parameter)

        # linearly interpolate the ankle angle
        new_angles = [Setpoint.get_angles_from_pos(new_foot_pos[0], 'left'),
                      Setpoint.get_angles_from_pos(new_foot_pos[1], 'right')]
        new_ankle_pos = [base_setpoints['left_ankle'].position * (1 - parameter)
                         + other_setpoints['left_ankle'].position * parameter,
                         base_setpoints['right_ankle'].position * (1 - parameter)
                         + other_setpoints['right_ankle'].position * parameter]
        new_vel = [Setpoint.get_angles_from_pos(new_foot_vel[0], 'left'),
                   Setpoint.get_angles_from_pos(new_foot_vel[1], 'right')]
        new_ankle_vel = [base_setpoints['left_ankle'].velocity * (1 - parameter)
                         + other_setpoints['left_ankle'].velocity * parameter,
                         base_setpoints['right_ankle'].velocity * (1 - parameter)
                         + other_setpoints['right_ankle'].velocity * parameter]

        base_setpoints_time = 0
        other_setpoints_time = 0
        for setpoint in base_setpoints.values():
            base_setpoints_time += setpoint.time
        for setpoint in other_setpoints.values():
            other_setpoints_time += setpoint.time
        time = (base_setpoints_time * (1 - parameter) + other_setpoints_time * parameter) / len(base_setpoints)

        resulting_setpoints = {'left_hip_aa': cls(time, new_angles[0][0], new_vel[0][0]),
                               'left_hip_fe': cls(time, new_angles[0][1], new_vel[0][1]),
                               'left_knee': cls(time, new_angles[0][2], new_vel[0][2]),
                               'right_hip_aa': cls(time, new_angles[1][0], new_vel[1][0]),
                               'right_hip_fe': cls(time, new_angles[1][1], new_vel[1][1]),
                               'right_knee': cls(time, new_angles[1][2], new_vel[1][2]),
                               'left_ankle': cls(time, new_ankle_pos[0], new_ankle_vel[0]),
                               'right_ankle': cls(time, new_ankle_pos[1], new_ankle_vel[1])}

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
        time = parameter * base_setpoint.time + (1 - parameter) * other_setpoint.time
        position = parameter * base_setpoint.position + (1 - parameter) * other_setpoint.position
        velocity = parameter * base_setpoint.velocity + (1 - parameter) * other_setpoint.velocity
        return Setpoint(time, position, velocity)

    @staticmethod
    def get_foot_pos_from_angles(setpoint_dic, velocity=False):
        """ calculate the position of the foot (ankle, ADFP is not taken into account) from joint angles. The origin of
        the local coordinate system is in the rotation point of the haa joint of the corresponding foot"""

        if velocity:
            l_haa = setpoint_dic['left_hip_aa'].velocity
            l_hfe = setpoint_dic['left_hip_fe'].velocity
            l_kfe = setpoint_dic['left_knee'].velocity
            r_haa = setpoint_dic['right_hip_aa'].velocity
            r_hfe = setpoint_dic['right_hip_fe'].velocity
            r_kfe = setpoint_dic['right_knee'].velocity
        else:
            l_haa = setpoint_dic['left_hip_aa'].position
            l_hfe = setpoint_dic['left_hip_fe'].position
            l_kfe = setpoint_dic['left_knee'].position
            r_haa = setpoint_dic['right_hip_aa'].position
            r_hfe = setpoint_dic['right_hip_fe'].position
            r_kfe = setpoint_dic['right_knee'].position
            if l_haa > pi or l_haa < - pi or r_haa > pi or r_haa < - pi or l_hfe > pi or l_hfe < - pi or l_kfe > pi \
                    or l_kfe < 0 or r_kfe > pi or r_kfe < 0:
                raise SubgaitInterpolationError("Angles do not adhere to the hard limits: l_haa = {0}, l_hfe = {1}, "
                                                "l_kfe = {2}, r_haa = {3}, r_hfe = {4}, r_kfe = {5}".
                                                format(l_haa, l_hfe, l_kfe, r_haa, r_hfe, r_kfe))

        robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')

        ul_l = robot.link_map['upper_leg_left'].collisions[0].geometry.size[2]
        ll_l = robot.link_map['lower_leg_left'].collisions[0].geometry.size[2]
        bb_l = robot.link_map['hip_aa_frame_left_front'].collisions[0].geometry.size[0]
        ph_l = robot.link_map['hip_aa_frame_left_side'].collisions[0].geometry.size[1]
        ul_r = robot.link_map['upper_leg_right'].collisions[0].geometry.size[2]
        ll_r = robot.link_map['lower_leg_right'].collisions[0].geometry.size[2]
        bb_r = robot.link_map['hip_aa_frame_right_front'].collisions[0].geometry.size[0]
        ph_r = robot.link_map['hip_aa_frame_right_side'].collisions[0].geometry.size[1]
        base = robot.link_map['hip_base'].collisions[0].geometry.size[1]

        # x is positive in the walking direction, z is in the downward direction, y is directed to the right side
        # the origin in the middle of the hip structure
        haa_to_foot_length_left = ul_l * cos(l_hfe) + ll_l * cos(l_hfe - l_kfe)
        left_y = - cos(l_haa) * ph_l - sin(l_haa) * haa_to_foot_length_left - base / 2.0
        left_z = - sin(l_haa) * ph_l + cos(l_haa) * haa_to_foot_length_left
        left_x = bb_l + sin(l_hfe) * ul_l + sin(l_hfe - l_kfe) * ll_l

        haa_to_foot_length_right = ul_r * cos(r_hfe) + ll_r * cos(r_hfe - r_kfe)
        right_y = cos(r_haa) * ph_r + sin(r_haa) * haa_to_foot_length_right + base / 2.0
        right_z = - sin(r_haa) * ph_r + cos(r_haa) * haa_to_foot_length_right
        right_x = bb_r + sin(r_hfe) * ul_r + sin(r_hfe - r_kfe) * ll_r

        left_foot_pos = [left_x, left_y, left_z]
        right_foot_pos = [right_x, right_y, right_z]

        return [left_foot_pos, right_foot_pos]

    @staticmethod
    def get_angles_from_pos(position, foot):
        """Calculates the angles of the joints corresponding to a certain position of the right and left foot
        w.r.t. the origin in the rotation point of the haa"""

        robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')
        if foot == 'left':
            ul = robot.link_map['upper_leg_left'].collisions[0].geometry.size[2]
            ll = robot.link_map['lower_leg_left'].collisions[0].geometry.size[2]
            bb = robot.link_map['hip_aa_frame_left_front'].collisions[0].geometry.size[0]
            ph = robot.link_map['hip_aa_frame_left_side'].collisions[0].geometry.size[1]
        else:
            ul = robot.link_map['upper_leg_right'].collisions[0].geometry.size[2]
            ll = robot.link_map['lower_leg_right'].collisions[0].geometry.size[2]
            bb = robot.link_map['hip_aa_frame_right_front'].collisions[0].geometry.size[0]
            ph = robot.link_map['hip_aa_frame_right_side'].collisions[0].geometry.size[1]
        base = robot.link_map['hip_base'].collisions[0].geometry.size[1]

        pos_x = position[0]
        # change y positive direction to go to the foot, easier for calculation, change origin to pivot of haa joint
        if foot == 'left':
            pos_y = - (position[1] + base / 2.0)
        else:
            pos_y = position[1] - base / 2.0
        pos_z = position[2]

        # first calculate the haa angle. This calculation assumes that pos_z > 0
        if pos_z <= 0:
            raise SubgaitInterpolationError("desired z_pos is less then zero, current inverse kinematic calculation is"
                                            " not capable of this")
        if pos_y != 0:
            slope_y_to_or = pos_z / pos_y
            alpha = atan(slope_y_to_or)
            if pos_y > 0:
                haa = acos(ph / sqrt(pos_z * pos_z + pos_y * pos_y)) - alpha
            else:
                haa = acos(ph / sqrt(pos_z * pos_z + pos_y * pos_y)) - pi - alpha
        else:
            alpha = pi / 2
            haa = acos(ph / sqrt(pos_z * pos_z + pos_y * pos_y)) - alpha


        # once the haa angle is known, use https://www.wolframalpha.com/input/?i=solve+%5Bsin%28x%29+%2B+sin%28x+-+y%29
        # *c%2C+cos%28x%29+%2B+cos%28x+-+y%29*c%5D+%3D+%5Ba%2C+b%5D to calculate the angles of the hfe and kfe
        # rescale for easier solving, and check if position is valid
        rescaled_x = round(pos_x - bb, 10)
        rescaled_z = round(sqrt(- ph * ph + pos_y * pos_y + pos_z * pos_z), 10)
        ll = ll / ul
        rescaled_x = rescaled_x / ul
        rescaled_z = rescaled_z / ul
        ul = 1.0

        if rescaled_x * rescaled_x + rescaled_z * rescaled_z > (ll + ul) * (ll + ul):
            raise SubgaitInterpolationError("The desired foot position, ({0}, {1}, {2}), is out of reach".
                                            format(position[0], position[1], position[2]))

        # make the calculation more concise
        try:
            big_sqrt_plus = sqrt(-rescaled_x * rescaled_x - rescaled_z * rescaled_z + ll * ll + 2 * ll + 1)
            big_sqrt_min = sqrt(rescaled_x * rescaled_x + rescaled_z * rescaled_z - ll * ll + 2 * ll - 1)
            denom_hip = (rescaled_x * rescaled_x + rescaled_z * rescaled_z + 2 * rescaled_z - ll * ll + 1) \
                        * big_sqrt_min
            numer_op_one = - rescaled_x * rescaled_x * big_sqrt_plus + 2 * rescaled_x * big_sqrt_min - rescaled_z \
                           * rescaled_z \
                           * big_sqrt_plus + ll * ll * big_sqrt_plus - 2 * ll * big_sqrt_plus + big_sqrt_plus
            numer_op_two = rescaled_x * rescaled_x * big_sqrt_plus + 2 * rescaled_x * big_sqrt_min + rescaled_z \
                           * rescaled_z \
                           * big_sqrt_plus - ll * ll * big_sqrt_plus + 2 * ll * big_sqrt_plus - big_sqrt_plus
            safety_check_large_op_one = ll * (rescaled_x * rescaled_x * rescaled_z * big_sqrt_min + 2 * rescaled_x
                                              * rescaled_x
                                              * big_sqrt_min - rescaled_x * rescaled_z * big_sqrt_plus + rescaled_x
                                              * ll * ll
                                              * big_sqrt_plus - 2 * rescaled_x * ll * big_sqrt_plus + rescaled_x *
                                              big_sqrt_plus +
                                              2 * rescaled_z * rescaled_z * big_sqrt_min - rescaled_z * ll * ll
                                              * big_sqrt_min
                                              + rescaled_z * big_sqrt_min + rescaled_z * rescaled_z * rescaled_z
                                              * big_sqrt_min
                                              - rescaled_x * rescaled_x * rescaled_x * big_sqrt_plus)
            safety_check_large_op_two = - ll * (rescaled_x * rescaled_x * rescaled_z * big_sqrt_min - 2 * rescaled_x
                                                * rescaled_x
                                                * big_sqrt_min - rescaled_x * rescaled_z * big_sqrt_plus + rescaled_x
                                                * ll * ll
                                                * big_sqrt_plus - 2 * rescaled_x * ll * big_sqrt_plus + rescaled_x *
                                                big_sqrt_plus -
                                                2 * rescaled_z * rescaled_z * big_sqrt_min + rescaled_z * ll * ll
                                                * big_sqrt_min
                                                - rescaled_z * big_sqrt_min - rescaled_z * rescaled_z * rescaled_z
                                                * big_sqrt_min
                                                - rescaled_x * rescaled_x * rescaled_x * big_sqrt_plus)
        except:
            raise SubgaitInterpolationError("The calculation method cannot find the angles corresponding to the desired"
                                            " foot position, ({0}, {1}, {2}).".
                                            format(pos_x, pos_y, pos_z))

        if rescaled_x * rescaled_x + rescaled_z * rescaled_z - ll * ll + 2 * rescaled_z - 1 == 0 or big_sqrt_min == 0 \
                or safety_check_large_op_one == 0 or safety_check_large_op_two == 0:
            raise SubgaitInterpolationError("The calculation method cannot find the angles corresponding to the desired"
                                            "foot position, ({0}, {1}, {2}).".
                                            format(pos_x, pos_y, pos_z))

        # calculate suitable hfe and kfe angles
        if denom_hip == 0:
            if numer_op_one == 0:
                if rescaled_x < 0:
                    hfe_one = -pi / 2
                else:
                    hfe_one = pi / 2
            elif numer_op_one < 0:
                hfe_one = - pi
            else:
                hfe_one = pi

            if numer_op_two == 0:
                if rescaled_x < 0:
                    hfe_two = -pi / 2
                else:
                    hfe_two = pi / 2
            elif numer_op_two < 0:
                hfe_two = - pi
            else:
                hfe_two = pi
        else:
            hfe_one = 2 * atan(numer_op_one / denom_hip)
            hfe_two = 2 * atan(numer_op_two / denom_hip)

        kfe_one = - 2 * atan(big_sqrt_plus / big_sqrt_min)
        kfe_two = 2 * atan(big_sqrt_plus / big_sqrt_min)

        if kfe_one > 0:
            kfe = kfe_one
            hfe = hfe_one
        else:
            kfe = kfe_two
            hfe = hfe_two

        return [haa, hfe, kfe]


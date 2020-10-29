from math import acos, atan, cos, pi, sin, sqrt

import numpy as np
import rospkg
import rospy
from urdf_parser_py import urdf

from march_shared_classes.exceptions.gait_exceptions import SubgaitInterpolationError

VELOCITY_SCALE = 250


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
    def create_position_interpolated_setpoints(cls, base_setpoints, other_setpoints, parameter):
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
        base_foot_pos = np.array(Setpoint.get_foot_pos_from_angles(base_setpoints))
        base_foot_vel = np.array(Setpoint.get_foot_pos_from_angles(base_setpoints, velocity=True))
        other_foot_pos = np.array(Setpoint.get_foot_pos_from_angles(other_setpoints))
        other_foot_vel = np.array(Setpoint.get_foot_pos_from_angles(other_setpoints, velocity=True))

        new_foot_pos = Setpoint.weighted_average(base_foot_pos, other_foot_pos, parameter)
        new_foot_vel = Setpoint.weighted_average(base_foot_vel, other_foot_vel, parameter)

        new_angles_pos = [Setpoint.get_angles_from_pos(new_foot_pos[0], 'left'),
                          Setpoint.get_angles_from_pos(new_foot_pos[1], 'right')]
        # Calculate new velocity by finding the foot position one 250th (one ehtercat cycle) of a second later.
        new_angles_vel = (- np.array(new_angles_pos)
                          + np.array([Setpoint.get_angles_from_pos(new_foot_pos[0] + new_foot_vel[0]
                                                                   / VELOCITY_SCALE, 'left'),
                                      Setpoint.get_angles_from_pos(new_foot_pos[1] + new_foot_vel[1]
                                                                   / VELOCITY_SCALE, 'right')])) \
            * VELOCITY_SCALE

        # linearly interpolate the ankle angle, as it cannot be calculated from the inverse kinematics
        new_ankle_pos = [Setpoint.weighted_average(base_setpoints['left_ankle'].position,
                                                   other_setpoints['left_ankle'].position, parameter),
                         Setpoint.weighted_average(base_setpoints['right_ankle'].position,
                                                   other_setpoints['right_ankle'].position, parameter)]
        new_ankle_vel = [Setpoint.weighted_average(base_setpoints['left_ankle'].velocity,
                                                   other_setpoints['left_ankle'].velocity, parameter),
                         Setpoint.weighted_average(base_setpoints['right_ankle'].velocity,
                                                   other_setpoints['right_ankle'].velocity, parameter)]

        # Set the time of the new setpoints as the weighted average of the original setpoint times
        base_setpoints_time = 0
        other_setpoints_time = 0
        for setpoint in base_setpoints.values():
            base_setpoints_time += setpoint.time
        for setpoint in other_setpoints.values():
            other_setpoints_time += setpoint.time
        time = Setpoint.weighted_average(base_setpoints_time, other_setpoints_time, parameter) / len(base_setpoints)

        resulting_setpoints = {'left_hip_aa': cls(time, new_angles_pos[0][0], new_angles_vel[0][0]),
                               'left_hip_fe': cls(time, new_angles_pos[0][1], new_angles_vel[0][1]),
                               'left_knee': cls(time, new_angles_pos[0][2], new_angles_vel[0][2]),
                               'right_hip_aa': cls(time, new_angles_pos[1][0], new_angles_vel[1][0]),
                               'right_hip_fe': cls(time, new_angles_pos[1][1], new_angles_vel[1][1]),
                               'right_knee': cls(time, new_angles_pos[1][2], new_angles_vel[1][2]),
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
            the foot location or velocity as a 2x3 list. Origin in the hip base, y positive to the right.
        """
        l_haa = setpoint_dic['left_hip_aa'].position
        l_hfe = setpoint_dic['left_hip_fe'].position
        l_kfe = setpoint_dic['left_knee'].position
        r_haa = setpoint_dic['right_hip_aa'].position
        r_hfe = setpoint_dic['right_hip_fe'].position
        r_kfe = setpoint_dic['right_knee'].position

        # get lengths from robot model, l_ul = left upper leg etc. see get_lengths_robot().
        l_ul, l_ll, l_bb, l_ph, r_ul, r_ll, r_bb, r_ph, base = Setpoint.get_lengths_robot()

        left_x, left_y, left_z = Setpoint.calculate_foot_position(l_haa, l_hfe, l_kfe, base, l_ph,
                                                                  l_bb, l_ul, l_ll, 'left')
        right_x, right_y, right_z = Setpoint.calculate_foot_position(r_haa, r_hfe, r_kfe, base, r_ph,
                                                                     r_bb, r_ul, r_ll, 'right')

        if velocity:
            # To calculate the velocity of the foot, find the foot location as it would be one ethercat cycle later.
            # The velocities need to be scaled as the position one second later could be invalid.
            l_haa_next = l_haa + setpoint_dic['left_hip_aa'].velocity / VELOCITY_SCALE
            l_hfe_next = l_hfe + setpoint_dic['left_hip_fe'].velocity / VELOCITY_SCALE
            l_kfe_next = l_kfe + setpoint_dic['left_knee'].velocity / VELOCITY_SCALE
            r_haa_next = r_haa + setpoint_dic['right_hip_aa'].velocity / VELOCITY_SCALE
            r_hfe_next = r_hfe + setpoint_dic['right_hip_fe'].velocity / VELOCITY_SCALE
            r_kfe_next = r_kfe + setpoint_dic['right_knee'].velocity / VELOCITY_SCALE

            left_x_next, left_y_next, left_z_next = Setpoint.calculate_foot_position(l_haa_next, l_hfe_next,
                                                                                     l_kfe_next, base, l_ph, l_bb,
                                                                                     l_ul, l_ll, 'left')
            right_x_next, right_y_next, right_z_next = Setpoint.calculate_foot_position(r_haa_next, r_hfe_next,
                                                                                        r_kfe_next, base, r_ph, r_bb,
                                                                                        r_ul, r_ll, 'right')

            # Rescale the velocities back to radians per second.
            left_y_velocity = (left_y_next - left_y) * VELOCITY_SCALE
            left_z_velocity = (left_z_next - left_z) * VELOCITY_SCALE
            left_x_velocity = (left_x_next - left_x) * VELOCITY_SCALE

            right_y_velocity = (right_y_next - right_y) * VELOCITY_SCALE
            right_z_velocity = (right_z_next - right_z) * VELOCITY_SCALE
            right_x_velocity = (right_x_next - right_x) * VELOCITY_SCALE

            left_foot_velocity = [left_x_velocity, left_y_velocity, left_z_velocity]
            right_foot_velocity = [right_x_velocity, right_y_velocity, right_z_velocity]

            return [left_foot_velocity, right_foot_velocity]
        else:
            left_foot_position = [left_x, left_y, left_z]
            right_foot_position = [right_x, right_y, right_z]

            return [left_foot_position, right_foot_position]

    @staticmethod
    def calculate_foot_position(haa, hfe, kfe, base, ph, bb, ul, ll, foot):
        """Calculates the foot position given the relevant angles, lengths and a specification of the foot."""
        # x is positive in the walking direction, z is in the downward direction, y is directed to the right side
        # the origin in the middle of the hip structure. The calculations are supported by
        # https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
        haa_to_foot_length = ul * cos(hfe) + ll * cos(hfe - kfe)
        z_position = - sin(haa) * ph + cos(haa) * haa_to_foot_length
        x_position = bb + sin(hfe) * ul + sin(hfe - kfe) * ll
        if foot == 'left':
            y_position = - cos(haa) * ph - sin(haa) * haa_to_foot_length - base / 2.0
        elif foot == 'right':
            y_position = cos(haa) * ph + sin(haa) * haa_to_foot_length + base / 2.0
        else:
            rospy.logwarn('invalid foot specified, {0} was given, does not match "left" or "right"'.format(foot))
            return
        return [x_position, y_position, z_position]

    @staticmethod
    def get_angles_from_pos(position, foot):
        """Calculates the angles of the joints corresponding to a certain position of the right or left foot.

        :param position:
            List that specified the x, y and z position of the foot. Origin in the hip base, y positive to the right
        :param foot:
            String that specifies to which foot the coordinates in position belong

        :return:
            Haa, kfe and hfe angles which correspond to the given position
        """
        # change y positive direction to the desired foot, change origin to pivot of haa joint, for ease of calculation
        # get lengths from robot model, ul = upper leg etc. see get_lengths_robot().
        pos_x = position[0]
        if foot == 'left':
            [ul, ll, bb, ph, base] = Setpoint.get_lengths_robot('left')
            pos_y = - (position[1] + base / 2.0)
        elif foot == 'right':
            [ul, ll, bb, ph, base] = Setpoint.get_lengths_robot('right')
            pos_y = position[1] - base / 2.0
        else:
            rospy.logwarn('invalid foot specified, {0} was given, does not match "left" or "right"'.format(foot))
        pos_z = position[2]

        # first calculate the haa angle. This calculation assumes that pos_z > 0, for details see
        # https://confluence.projectmarch.nl:8443/display/62tech/Inverse+kinematics
        if pos_z <= 0:
            raise SubgaitInterpolationError('desired z_pos is not positive, current inverse kinematic calculation is'
                                            ' not capable of this')
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
            raise SubgaitInterpolationError('The desired foot position, ({0}, {1}, {2}), is out of reach'.
                                            format(position[0], position[1], position[2]))

        # make the calculation more concise
        try:
            big_sqrt_plus = sqrt(-rescaled_x * rescaled_x - rescaled_z * rescaled_z + ll * ll + 2 * ll + 1)
            big_sqrt_min = sqrt(rescaled_x * rescaled_x + rescaled_z * rescaled_z - ll * ll + 2 * ll - 1)
            denom_hip = (rescaled_x * rescaled_x + rescaled_z * rescaled_z + 2 * rescaled_z - ll * ll + 1) * \
                big_sqrt_min
            numer_op_one = - rescaled_x * rescaled_x * big_sqrt_plus + 2 * rescaled_x * big_sqrt_min - rescaled_z * \
                rescaled_z * big_sqrt_plus + ll * ll * big_sqrt_plus - 2 * ll * big_sqrt_plus + big_sqrt_plus
            numer_op_two = rescaled_x * rescaled_x * big_sqrt_plus + 2 * rescaled_x * big_sqrt_min + rescaled_z * \
                rescaled_z * big_sqrt_plus - ll * ll * big_sqrt_plus + 2 * ll * big_sqrt_plus - big_sqrt_plus
            safety_check_large_op_one = ll * (rescaled_x * rescaled_x * rescaled_z * big_sqrt_min + 2 * rescaled_x
                                              * rescaled_x * big_sqrt_min - rescaled_x * rescaled_z * big_sqrt_plus
                                              + rescaled_x * ll * ll * big_sqrt_plus - 2 * rescaled_x * ll
                                              * big_sqrt_plus + rescaled_x * big_sqrt_plus + 2 * rescaled_z
                                              * rescaled_z * big_sqrt_min - rescaled_z * ll * ll * big_sqrt_min
                                              + rescaled_z * big_sqrt_min + rescaled_z * rescaled_z * rescaled_z
                                              * big_sqrt_min - rescaled_x * rescaled_x * rescaled_x * big_sqrt_plus)
            safety_check_large_op_two = - ll * (rescaled_x * rescaled_x * rescaled_z * big_sqrt_min - 2 * rescaled_x
                                                * rescaled_x * big_sqrt_min - rescaled_x * rescaled_z * big_sqrt_plus
                                                + rescaled_x * ll * ll * big_sqrt_plus - 2 * rescaled_x * ll
                                                * big_sqrt_plus + rescaled_x * big_sqrt_plus - 2 * rescaled_z
                                                * rescaled_z * big_sqrt_min + rescaled_z * ll * ll * big_sqrt_min
                                                - rescaled_z * big_sqrt_min - rescaled_z * rescaled_z * rescaled_z
                                                * big_sqrt_min - rescaled_x * rescaled_x * rescaled_x * big_sqrt_plus)
        except ValueError:
            raise SubgaitInterpolationError('The calculation method cannot find the angles corresponding to the desired'
                                            ' foot position, ({0}, {1}, {2}).'.
                                            format(pos_x, pos_y, pos_z))

        # Make sure the desired position adheres to the limits of the calculation given by wolfram alpha
        if rescaled_x * rescaled_x + rescaled_z * rescaled_z - ll * ll + 2 * rescaled_z - 1 == 0 or big_sqrt_min == 0 \
                or safety_check_large_op_one == 0 or safety_check_large_op_two == 0:
            raise SubgaitInterpolationError('The calculation method cannot find the angles corresponding to the desired'
                                            'foot position, ({0}, {1}, {2}).'.
                                            format(pos_x, pos_y, pos_z))

        # calculate the hfe and kfe angles, since there are two options for these angles, pick the one that with
        # positive knee flexion.
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

    @staticmethod
    def weighted_average(base_value, other_value, parameter):
        """Compute the weighted average of two values with normalised weight parameter."""
        return base_value * (1 - parameter) + other_value * parameter

    @staticmethod
    def get_lengths_robot(foot=''):
        robot = urdf.Robot.from_xml_file(rospkg.RosPack().get_path('march_description') + '/urdf/march4.urdf')
        l_ul = robot.link_map['upper_leg_left'].collisions[0].geometry.size[2]  # left upper leg length
        l_ll = robot.link_map['lower_leg_left'].collisions[0].geometry.size[2]  # left lower leg length
        l_bb = robot.link_map['hip_aa_frame_left_front'].collisions[0].geometry.size[0]  # left haa arm to leg
        # (billen been)
        l_ph = robot.link_map['hip_aa_frame_left_side'].collisions[0].geometry.size[1]  # left pelvic hip length
        r_ul = robot.link_map['upper_leg_right'].collisions[0].geometry.size[2]  # right upper leg length
        r_ll = robot.link_map['lower_leg_right'].collisions[0].geometry.size[2]  # right lower leg length
        r_bb = robot.link_map['hip_aa_frame_right_front'].collisions[0].geometry.size[0]  # right haa arm to leg
        # (billen been)
        r_ph = robot.link_map['hip_aa_frame_right_side'].collisions[0].geometry.size[1]  # right pelvic hip length
        base = robot.link_map['hip_base'].collisions[0].geometry.size[1]
        if foot == 'left':
            return [l_ul, l_ll, l_bb, l_ph, base]
        elif foot == 'right':
            return [r_ul, r_ll, r_bb, r_ph, base]
        elif foot == '':
            return [l_ul, l_ll, l_bb, l_ph, r_ul, r_ll, r_bb, r_ph, base]
        else:
            rospy.logwarn('invalid foot specified, {0} was given, does not match "left" or "right"'.format(foot))

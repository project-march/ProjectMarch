from math import acos, atan, cos, pi, sin, sqrt

import rospkg
from urdf_parser_py import urdf

from march_shared_classes.exceptions.gait_exceptions import SideSpecificationError, SubgaitInterpolationError

# Use this factor when calculating velocities to keep the calculations within the range of motion
# See IK confluence page https://confluence.projectmarch.nl:8443/display/62tech/Inverse+kinematics
VELOCITY_SCALE_FACTOR = 500


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
                                                                             velocity=new_foot_vel)
        new_angles_right = Setpoint.calculate_joint_angles_from_foot_position(new_foot_pos, 'right',
                                                                              velocity=new_foot_vel)

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
        try:
            l_haa = setpoint_dic['left_hip_aa'].position
            l_hfe = setpoint_dic['left_hip_fe'].position
            l_kfe = setpoint_dic['left_knee'].position
            r_haa = setpoint_dic['right_hip_aa'].position
            r_hfe = setpoint_dic['right_hip_fe'].position
            r_kfe = setpoint_dic['right_knee'].position
        except KeyError as e:
            raise KeyError('Expected setpoint dictionary to contain "{key}", but "{key}" was missing.'.
                           format(key=e.args[0]))

        left_foot = Setpoint.calculate_foot_position(l_haa, l_hfe, l_kfe, 'left')
        right_foot = Setpoint.calculate_foot_position(r_haa, r_hfe, r_kfe, 'right')

        if velocity:
            # To calculate the velocity of the foot, find the foot location as it would be a fraction of a second later.
            # The velocities need to be scaled as the position one second later could be invalid.
            l_haa_next = l_haa + setpoint_dic['left_hip_aa'].velocity / VELOCITY_SCALE_FACTOR
            l_hfe_next = l_hfe + setpoint_dic['left_hip_fe'].velocity / VELOCITY_SCALE_FACTOR
            l_kfe_next = l_kfe + setpoint_dic['left_knee'].velocity / VELOCITY_SCALE_FACTOR
            r_haa_next = r_haa + setpoint_dic['right_hip_aa'].velocity / VELOCITY_SCALE_FACTOR
            r_hfe_next = r_hfe + setpoint_dic['right_hip_fe'].velocity / VELOCITY_SCALE_FACTOR
            r_kfe_next = r_kfe + setpoint_dic['right_knee'].velocity / VELOCITY_SCALE_FACTOR

            left_foot_next = Setpoint.calculate_foot_position(l_haa_next, l_hfe_next, l_kfe_next, 'left')
            right_foot_next = Setpoint.calculate_foot_position(r_haa_next, r_hfe_next, r_kfe_next, 'right')

            # Rescale the velocities back to radians per second.
            left_y_velocity = (left_foot_next['y'] - left_foot['y']) * VELOCITY_SCALE_FACTOR
            left_z_velocity = (left_foot_next['z'] - left_foot['z']) * VELOCITY_SCALE_FACTOR
            left_x_velocity = (left_foot_next['x'] - left_foot['x']) * VELOCITY_SCALE_FACTOR

            right_y_velocity = (right_foot_next['y'] - right_foot['y']) * VELOCITY_SCALE_FACTOR
            right_z_velocity = (right_foot_next['z'] - right_foot['z']) * VELOCITY_SCALE_FACTOR
            right_x_velocity = (right_foot_next['x'] - right_foot['x']) * VELOCITY_SCALE_FACTOR

            velocity_dictionary = {'left_x_velocity': left_x_velocity, 'left_y_velocity': left_y_velocity,
                                   'left_z_velocity': left_z_velocity, 'right_x_velocity': right_x_velocity,
                                   'right_y_velocity': right_y_velocity, 'right_z_velocity': right_z_velocity}
            return velocity_dictionary

        else:
            position_dictionary = {'left_foot_x': left_foot['x'], 'left_foot_y': left_foot['y'],
                                   'left_foot_z': left_foot['z'], 'right_foot_x': right_foot['x'],
                                   'right_foot_y': right_foot['y'], 'right_foot_z': right_foot['z']}
            return position_dictionary

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
        elif side == 'right':
            y_position = cos(haa) * ph + sin(haa) * haa_to_foot_length + base / 2.0
        else:
            raise SideSpecificationError(side)

        return {'x': x_position, 'y': y_position, 'z': z_position}

    @staticmethod
    def calculate_joint_angles_from_foot_position(position, foot, velocity=None):
        """Calculates the angles of the joints corresponding to a certain position of the right or left foot.

        More information on the calculations of the haa, hfe and kfe angles, aswel as on the velocity calculations can
        be found at https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics

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
            pos_x = position['left_foot_x']
            pos_z = position['left_foot_z']
            [ul, ll, hl, ph, base] = Setpoint.get_lengths_robot('left')
            pos_y = - (position['left_foot_y'] + base / 2.0)
            if bool(velocity):
                velocity_x = velocity['left_x_velocity']
                velocity_y = velocity['left_y_velocity']
                velocity_z = velocity['left_z_velocity']
                next_positions = {foot + '_foot_x': pos_x + velocity_x / VELOCITY_SCALE_FACTOR,
                                  foot + '_foot_y': - pos_y + velocity_y / VELOCITY_SCALE_FACTOR - base / 2.0,
                                  foot + '_foot_z': pos_z + velocity_z / VELOCITY_SCALE_FACTOR}
        elif foot == 'right':
            pos_x = position['right_foot_x']
            pos_z = position['right_foot_z']
            [ul, ll, hl, ph, base] = Setpoint.get_lengths_robot('right')
            pos_y = position['right_foot_y'] - base / 2.0
            if bool(velocity):
                velocity_x = velocity['right_x_velocity']
                velocity_y = velocity['right_y_velocity']
                velocity_z = velocity['right_z_velocity']
                next_positions = {foot + '_foot_x': pos_x + velocity_x / VELOCITY_SCALE_FACTOR,
                                  foot + '_foot_y': pos_y + velocity_y / VELOCITY_SCALE_FACTOR + base / 2.0,
                                  foot + '_foot_z': pos_z + velocity_z / VELOCITY_SCALE_FACTOR}
        else:
            raise SideSpecificationError(foot)

        # first calculate the haa angle. This calculation assumes that pos_z > 0, for details see
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
        rescaled_x = round(pos_x - hl, 10)
        rescaled_z = round(sqrt(- ph * ph + pos_y * pos_y + pos_z * pos_z), 10)
        ll = ll / ul
        rescaled_x = rescaled_x / ul
        rescaled_z = rescaled_z / ul
        ul = 1.0

        if rescaled_x * rescaled_x + rescaled_z * rescaled_z > (ll + ul) * (ll + ul):
            raise SubgaitInterpolationError('The desired foot position, ({0}, {1}, {2}), is out of reach'.
                                            format(pos_x, pos_y, pos_z))

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

        angle_positions = {foot + '_hip_aa': haa, foot + '_hip_fe': hfe, foot + '_knee': kfe}

        if bool(velocity):
            # find the angles a moment later using the foot position a moment later
            # scale the found velocity back to radians per second
            next_angles = Setpoint.calculate_joint_angles_from_foot_position(next_positions, foot)
            angle_velocities = {foot + '_hip_aa_velocity': (- haa + next_angles[foot + '_hip_aa'])
                                * VELOCITY_SCALE_FACTOR,
                                foot + '_hip_fe_velocity': (- hfe + next_angles[foot + '_hip_fe'])
                                * VELOCITY_SCALE_FACTOR,
                                foot + '_knee_velocity': (- kfe + next_angles[foot + '_knee']) * VELOCITY_SCALE_FACTOR}
            angle_velocities.update(angle_positions)
            return angle_velocities

        return angle_positions

    @staticmethod
    def weighted_average_dictionary(base_dictionary, other_dictionary, parameter):
        resulting_dictionary = {}
        for key in base_dictionary.keys():
            try:
                resulting_dictionary[key] = Setpoint.weighted_average(base_dictionary[key],
                                                                      other_dictionary[key], parameter)
            except KeyError as e:
                raise KeyError('dictionaries must have the same keys for a weighted average. other_dictionary misses '
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
            l_ph = robot.link_map['hip_aa_frame_left_side'].collisions[0].geometry.size[1]  # left pelvic hip length
            r_ul = robot.link_map['upper_leg_right'].collisions[0].geometry.size[2]  # right upper leg length
            r_ll = robot.link_map['lower_leg_right'].collisions[0].geometry.size[2]  # right lower leg length
            r_hl = robot.link_map['hip_aa_frame_right_front'].collisions[0].geometry.size[0]  # right haa arm to leg
            r_ph = robot.link_map['hip_aa_frame_right_side'].collisions[0].geometry.size[1]  # right pelvic hip length
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

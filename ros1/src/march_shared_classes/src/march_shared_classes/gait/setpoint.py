from math import *
import numpy as np
from march_shared_classes.exceptions.gait_exceptions import SubgaitInterpolationError


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
    def interpolate_setpoints_position(base_setpoints, other_setpoints, parameter):
        """Linearly interpolate the ankle position.

        :param base_setpoints:
            A dictionary of setpoints, one for each joint. Return value if parameter is zero
        :param other_setpoints:
            A dictionary of setpoints, one for each joint. Return value if parameter is one
        :param parameter:
            parameter for the interpolation
        :return
            A list of setpoints, who's corresponding foot location is linearly interpolated from the setpoints"""

        base_foot_pos = np.array(get_foot_pos_from_angles(base_setpoints))
        other_foot_pos = np.array(get_foot_pos_from_angles(other_setpoints))
        new_foot_pos = base_foot_pos * (1 - parameter) + other_foot_pos * parameter
        new_angles = get_angles_from_pos(new_foot_pos)

        return base_setpoints[0]


def get_foot_pos_from_angles(setpoint_dic):
    """ calculated the position of the foot (ankle, ADFP is not taken into account) from joint angles"""

    l_haa = setpoint_dic['left_hip_aa']
    l_hfe = setpoint_dic['left_hip_fe']
    l_kfe = setpoint_dic['left_knee']
    r_haa = setpoint_dic['right_hip_aa']
    r_hfe = setpoint_dic['right_hip_fe']
    r_kfe = setpoint_dic['right_knee']

    bb = 1.0
    ll = 1.0
    ul = 1.0
    ph = 1.0

    # x is positive in the walking direction, z is in the downward direction, y is directed to the right side
    # the origin in the middle of the hip structure
    haa_to_foot_length = ul * cos(l_hfe) + ll * cos(l_hfe - l_kfe)
    left_x = cos(l_haa) * ph + sin(l_haa) * haa_to_foot_length
    left_y = sin(l_haa) * ph - cos(l_haa) * haa_to_foot_length
    left_z = bb + sin(l_hfe) + sin(l_hfe - l_kfe)

    right_x = cos(r_haa) * ph + sin(r_haa) * haa_to_foot_length
    right_y = sin(r_haa) * ph - cos(r_haa) * haa_to_foot_length
    right_z = bb + cos(r_hfe) + sin(r_hfe - r_kfe)

    left_foot_pos = [left_x, left_y, left_z]
    right_foot_pos = [right_x, right_y, right_z]

    return [left_foot_pos, right_foot_pos]


def get_angles_from_pos(position, foot):
    """Calculates the angles of the joints corresponding to a certain position of the right and left foot w.r.t. the
    origin in the middle of the hip"""

    pos_x = position[0]
    if foot == 'left':
        pos_y = - position[1]  # so the positive direction is to the outside, easier for calculation
    else:
        pos_y = position[1]
    pos_z = position[2]

    bb = 1.0
    ll = 1.0
    ul = 1.0
    ph = 1.0

    # first find the haa angle, this can be done by looking at the y-z plane with the knowledge that there must be a
    # straight line from the foot position tangent to the circle with r = ph around the origin
    if pos_y != 0:
        slope_y_to_or = pos_z / pos_y
        alpha = atan(slope_y_to_or)  # the angle with the y axis of that line
        if pos_y > 0:
            haa = acos(ph / sqrt(pos_z * pos_z + pos_y * pos_y)) - alpha
        else:
            haa = - acos(ph / sqrt(pos_z * pos_z + pos_y * pos_y)) + alpha
    else:
        alpha = pi / 2
        haa = acos(ph / sqrt(pos_z * pos_z + pos_y * pos_y)) - alpha

    # once the haa angle is known, use https://www.wolframalpha.com/input/?i=solve+%5Bsin%28x%29+%2B+sin%28x+-+y%29*c%2
    # C+cos%28x%29+%2B+cos%28x+-+y%29*c%5D+%3D+%5Ba%2C+b%5D to calculate the angles of the hfe and kfe
    # rescale for easier solving, and check if position is valid
    rescaled_x = pos_x - bb
    rescaled_z = (pos_z + sin(haa) * ph) * (pos_z + sin(haa) * ph) + (cos(haa) * ph - pos_y) * (cos(haa) * ph - pos_y)
    if sqrt(rescaled_x * rescaled_x + rescaled_z * rescaled_z) > ll + ul:
        raise SubgaitInterpolationError("The desired foot position, ({0}, {1}, {2}), is out of reach".
                                        format(pos_x, pos_y, pos_z))
    ll = ll / ul
    ul = 1.0
    rescaled_x = rescaled_x / ul
    rescaled_z = rescaled_z / ul

    # make the calculation more concise
    big_sqrt_plus = sqrt(-rescaled_x * rescaled_x - rescaled_z * rescaled_z + ll * ll + 1)
    big_sqrt_min = sqrt(rescaled_x * rescaled_x + rescaled_z * rescaled_z + ll * ll - 1)
    denom_hip = (rescaled_x * rescaled_x + rescaled_z * rescaled_z + 2 * rescaled_z - ll * ll + 1) * big_sqrt_min
    numer_op_one = - rescaled_x * rescaled_x * big_sqrt_plus + 2 * rescaled_x * big_sqrt_min - rescaled_z * rescaled_z \
                   * big_sqrt_plus + ll * ll * big_sqrt_plus - 2 * ll * big_sqrt_plus + big_sqrt_plus
    numer_op_two = rescaled_x * rescaled_x * big_sqrt_plus + 2 * rescaled_x * big_sqrt_min + rescaled_z * rescaled_z \
                   * big_sqrt_plus - ll * ll * big_sqrt_plus + 2 * ll * big_sqrt_plus - big_sqrt_plus
    safety_check_large = ll * (rescaled_x * rescaled_x * rescaled_z * big_sqrt_min + 2 * rescaled_x * rescaled_x
                               * big_sqrt_min - rescaled_x * rescaled_z * big_sqrt_plus + rescaled_x * ll * ll
                               * big_sqrt_plus - 2 * rescaled_x * ll * big_sqrt_plus + rescaled_x * big_sqrt_plus +
                               2 * rescaled_z * rescaled_z * big_sqrt_min - rescaled_z * ll * ll * big_sqrt_min
                               + rescaled_z * big_sqrt_min + rescaled_z * rescaled_z * rescaled_z * big_sqrt_min
                               - rescaled_x * rescaled_x * rescaled_x * big_sqrt_plus)

    if rescaled_x * rescaled_x + rescaled_z * rescaled_z - ll * ll + 2 * rescaled_z - 1 == 0 or big_sqrt_min == 0 \
            or safety_check_large == 0:
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

from march_shared_classes.utilities.utility_functions import merge_dictionaries, weighted_average

# Use this factor when calculating velocities to keep the calculations within the range of motion
# See IK confluence page https://confluence.projectmarch.nl:8443/display/62tech/%28Inverse%29+kinematics
VELOCITY_SCALE_FACTOR = 500


class Setpoint(object):
    """Base class to define the setpoints of a subgait."""

    digits = 4

    def __init__(self, time, position, velocity=None):
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

    def add_joint_velocity_from_next_angle(self, next_state):
        """Calculates the (left/right/all)joint velocities given a current position and a next position.

        Approximates using next_position = position + current_velocity * time_difference

        :param this: A Setpoint object with no velocity
        :param next_state: A Setpoint with the positions a moment later

        ":return: The joint velocities of the joints on the specified side
        """
        self.velocity = (next_state.position - self.position) * VELOCITY_SCALE_FACTOR

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
    def calculate_next_positions_joint(setpoint_dic):
        """Calculates the position of the joints a moment later given a setpoint dictionary.

        :param setpoint_dic: A dictionary of setpoints with positions and velocities
        :return: A dictionary with the positions of the joints 1 / VELOCITY_SCALE_FACTOR second later
        """
        next_positions = {}
        for joint in JOINT_NAMES_IK:
            if joint in setpoint_dic:
                next_positions[joint] = setpoint_dic[joint].position + setpoint_dic[joint].velocity \
                    / VELOCITY_SCALE_FACTOR
            else:
                raise KeyError('setpoint_dic is missing joint {joint}'.format(joint=joint))

        return next_positions

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

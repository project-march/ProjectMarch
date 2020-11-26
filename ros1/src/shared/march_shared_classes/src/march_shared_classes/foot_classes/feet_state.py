from march_shared_classes.exceptions.gait_exceptions import WeightedAverageError

from .foot import Foot
from march_shared_classes.utilities.utility_functions import merge_dictionaries, weighted_average


class FeetState(object):
    """Class for encapturing the entire state (position and velocity) of both feet."""

    def __init__(self, right_foot, left_foot, time=None):
        """Create a FeetState object, right_foot and left_foot are both Foot objects."""
        self.right_foot = right_foot
        self.left_foot = left_foot
        self.time = time

    @classmethod
    def weighted_average_states(cls, base_state, other_state, parameter):
        """Computes the weighted average of two feet states.

        :param base_state: One of the states for the weighted average, return this is parameter is 0
        :param other_state: One of the states for the weighted average, return this if parameter is 1
        :param parameter: The normalised weight parameter, the parameter that determines the weight of the other_state

        :return: A FeetState Object of which the positions and velocities of both the feet are the weighted average of
        those of the base and other states.
        """
        if parameter == 0:
            return base_state
        elif parameter == 1:
            return other_state

        if not(bool(base_state.right_foot.velocity) and bool(base_state.left_foot.velocity)
               and bool(other_state.right_foot.velocity) and bool(other_state.left_foot.velocity)):
            raise WeightedAverageError('Not all Foot objects of the FeetState object contain velocities, '
                                       'calculation cannot proceed')

        resulting_right_foot = Foot('right', weighted_average(base_state.right_foot.position,
                                                              other_state.right_foot.position, parameter),
                                    weighted_average(base_state.right_foot.velocity,
                                                     other_state.right_foot.velocity, parameter))
        resulting_left_foot = Foot('left', weighted_average(base_state.left_foot.position,
                                                            other_state.left_foot.position, parameter),
                                   weighted_average(base_state.left_foot.velocity,
                                                    other_state.left_foot.velocity, parameter))

        resulting_time = weighted_average(base_state.time, other_state.time, parameter)

        return cls(resulting_right_foot, resulting_left_foot, resulting_time)

    @staticmethod
    def feet_state_to_setpoint(feet_state):
        """Translates between feet_state and a list of setpoints, which correspond with the feet_state.

        :param feet_state: A fully populated FeetState object, with two fully populated Foot objects.

        :return: A dictionary of setpoints, the foot location and velocity of which corresponds with the feet_state
        """
        left_joint_states = Foot.get_joint_states_from_foot_state(feet_state.left_foot, feet_state.time)
        right_joint_states = Foot.get_joint_states_from_foot_state(feet_state.right_foot, feet_state.time)

        setpoints = merge_dictionaries(left_joint_states, right_joint_states)
        return setpoints

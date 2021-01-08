from march_shared_classes.gait.setpoint import Setpoint
from march_shared_classes.utilities.side import Side
from march_shared_classes.utilities.utility_functions import (
    get_joint_names_for_inverse_kinematics,
)
from march_shared_classes.utilities.utility_functions import (
    merge_dictionaries,
    weighted_average,
)

from .foot import Foot

JOINT_NAMES_IK = get_joint_names_for_inverse_kinematics()


class FeetState(object):
    """Class for encapturing the entire state (position and velocity at a certain time) of both feet."""

    def __init__(self, right_foot, left_foot, time=None):
        """Create a FeetState object, right_foot and left_foot are both Foot objects."""
        self.right_foot = right_foot
        self.left_foot = left_foot
        self.time = time

    @classmethod
    def from_setpoints(cls, setpoint_dic):
        """Calculate the position and velocity of the foot (or rather ankle) from joint angles.

        :param setpoint_dic:
            Dictionary of setpoints from which the feet positions and velocities need to be calculated, should all
            be around the same time

        :return:
            A FeetState object with a left and right foot which each have a position and velocity corresponding to the
            setpoint dictionary
        """
        for joint in JOINT_NAMES_IK:
            if joint not in setpoint_dic:
                raise KeyError(
                    "expected setpoint dictionary to contain joint {joint}, but {joint} was missing.".format(
                        joint=joint
                    )
                )

        foot_state_left = Foot.calculate_foot_position(
            setpoint_dic["left_hip_aa"].position,
            setpoint_dic["left_hip_fe"].position,
            setpoint_dic["left_knee"].position,
            Side.left,
        )
        foot_state_right = Foot.calculate_foot_position(
            setpoint_dic["right_hip_aa"].position,
            setpoint_dic["right_hip_fe"].position,
            setpoint_dic["right_knee"].position,
            Side.right,
        )

        next_joint_positions = Setpoint.calculate_next_positions_joint(setpoint_dic)

        next_foot_state_left = Foot.calculate_foot_position(
            next_joint_positions["left_hip_aa"].position,
            next_joint_positions["left_hip_fe"].position,
            next_joint_positions["left_knee"].position,
            Side.left,
        )
        next_foot_state_right = Foot.calculate_foot_position(
            next_joint_positions["right_hip_aa"].position,
            next_joint_positions["right_hip_fe"].position,
            next_joint_positions["right_knee"].position,
            Side.right,
        )

        foot_state_left.add_foot_velocity_from_next_state(next_foot_state_left)
        foot_state_right.add_foot_velocity_from_next_state(next_foot_state_right)

        # Set the time of the new setpoints as the weighted average of the original setpoint times
        feet_state_time = 0
        for setpoint in setpoint_dic.values():
            feet_state_time += setpoint.time
        feet_state_time = feet_state_time / len(setpoint_dic)

        # feet state
        return cls(foot_state_right, foot_state_left, feet_state_time)

    @classmethod
    def weighted_average_states(cls, base_state, other_state, parameter):
        """Computes the weighted average of two feet states.

        :param base_state: One of the states for the weighted average, return this if
                           parameter is 0
        :param other_state: One of the states for the weighted average, return this if
                            parameter is 1
        :param parameter: The normalised weight parameter, the parameter that determines
                          the weight of the other_state

        :return: A FeetState Object of which the positions and velocities of both the
                 feet are the weighted average of those of the base and other states.
        """
        if parameter == 0:
            return base_state
        elif parameter == 1:
            return other_state

        resulting_right_foot = Foot.weighted_average_foot(
            base_state.right_foot, other_state.right_foot, parameter
        )
        resulting_left_foot = Foot.weighted_average_foot(
            base_state.left_foot, other_state.left_foot, parameter
        )
        resulting_time = weighted_average(base_state.time, other_state.time, parameter)

        return cls(resulting_right_foot, resulting_left_foot, resulting_time)

    @staticmethod
    def feet_state_to_setpoints(feet_state):
        """Translates between feet_state and a list of setpoints, which correspond with the feet_state.

        :param feet_state: A fully populated FeetState object, with two fully populated Foot objects.

        :return: A dictionary of setpoints, the foot location and velocity of which corresponds with the feet_state
        """
        left_joint_states = Foot.get_joint_states_from_foot_state(
            feet_state.left_foot, feet_state.time
        )
        right_joint_states = Foot.get_joint_states_from_foot_state(
            feet_state.right_foot, feet_state.time
        )

        # setpoints
        return merge_dictionaries(left_joint_states, right_joint_states)

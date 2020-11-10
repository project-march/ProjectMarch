from .utilities import weighted_average_dictionaries
from .foot import Foot


class FeetState(object):
    def __init__(self, right_foot, left_foot):
        """Create a FeetState object, foot_1 and foot_2 are both Foot objects"""
        self.right_foot = right_foot
        self.left_foot = left_foot

    @classmethod
    def weighted_average_states(cls, base_state, other_state, parameter):
        resulting_right_foot = Foot('right', weighted_average_dictionaries(base_state.right_foot.position,
                                                                           other_state.right_foot.position, parameter),
                                    weighted_average_dictionaries(base_state.right_foot.velocity,
                                                                  other_state.right_foot.velocity, parameter))
        resulting_left_foot = Foot('left', weighted_average_dictionaries(base_state.left_foot.position,
                                                                         other_state.left_foot.position, parameter),
                                   weighted_average_dictionaries(base_state.left_foot.velocity,
                                                                 other_state.left_foot.velocity, parameter))

        return cls(resulting_right_foot, resulting_left_foot)

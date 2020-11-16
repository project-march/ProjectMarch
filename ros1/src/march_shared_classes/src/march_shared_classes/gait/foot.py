from march_shared_classes.exceptions.gait_exceptions import SideSpecificationError

VELOCITY_SCALE_FACTOR = 500


class Foot(object):
    """Class for capturing the state (position and possible velocity) of a foot."""

    def __init__(self, foot_side, position, velocity=None):
        """Create a Foot object, position and velocity are both Vector3d objects."""
        self.position = position
        self.velocity = velocity
        self.foot_side = foot_side
        if foot_side != 'left' and foot_side != 'right':
            raise SideSpecificationError(foot_side)

    def add_foot_velocity_from_next_state(self, next_state):
        """Adds the foot velocity to the state given a next state for the foot to be in.

        :param: next_state: A Foot object that specifies the foot location 1 / VELOCITY_SCALE_FACTOR second later

        :return: The object with a velocity which is calculated based on the next state
        """
        velocity = (next_state.position - self.position) * VELOCITY_SCALE_FACTOR
        self.velocity = velocity

    @classmethod
    def calculate_next_foot_position(cls, current_state):
        """Calculates the foot position a moment later given the current state.

        :param current_state: A Foot object with a velocity

        :return A Foot object with the position of the foot 1 / VELOCITY_SCALE_FACTOR second later
        """
        next_position = current_state.position + current_state.velocity / VELOCITY_SCALE_FACTOR
        return cls(current_state.foot_side, next_position)

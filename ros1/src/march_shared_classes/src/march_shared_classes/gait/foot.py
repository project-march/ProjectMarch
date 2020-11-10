from march_shared_classes.exceptions.gait_exceptions import SideSpecificationError, IncorrectCoordinateError

VELOCITY_SCALE_FACTOR = 500

class Foot(object):
    def __init__(self, foot_side, position, velocity=None):
        """Create a Foot object, position and velocity are both Coordinates."""
        self.position = position
        self.velocity = velocity
        self.foot_side = foot_side
        if foot_side != 'left' and foot_side != 'right':
            raise SideSpecificationError(foot_side)
        if {'x', 'y', 'z'} != set(self.position.keys()):
            raise IncorrectCoordinateError()
        if velocity is not None:
            if {'x', 'y', 'z'} != set(self.velocity.keys()):
                raise IncorrectCoordinateError()

    def add_foot_velocity_from_next_state(self, next_state):
        velocity = {}
        for key in self.position:
            velocity[key] = (next_state.position[key] - self.position[key]) * VELOCITY_SCALE_FACTOR
        self.velocity = velocity

    @classmethod
    def calculate_next_foot_position(cls, current_state):
        next_position = {}
        for key in current_state.position:
            next_position[key] = current_state.position[key] + current_state.velocity[key] / VELOCITY_SCALE_FACTOR
        return cls(current_state.foot_side, next_position)

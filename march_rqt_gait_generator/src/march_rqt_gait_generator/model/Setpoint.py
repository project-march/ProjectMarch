class Setpoint:

    def __init__(self, time, position, velocity):
        self.time = time
        self.position = position
        self.velocity = velocity

    def __repr__(self):
        return 'Time: %s, Position: %s, Velocity: %s' % (self.time, self.position, self.velocity)

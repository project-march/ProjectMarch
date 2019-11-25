from march_shared_classes.gait.setpoint import Setpoint


class ModifiableSetpoint(Setpoint):
    def invert(self, duration):
        self.time = round(duration - self.time, self.digits)
        self.velocity = round(-self.velocity, self.digits)

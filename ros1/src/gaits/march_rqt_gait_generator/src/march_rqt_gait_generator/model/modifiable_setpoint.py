from march_shared_classes.gait.setpoint import Setpoint


class ModifiableSetpoint(Setpoint):
    """ """

    def invert(self, duration):
        """

        Args:
          duration:

        Returns:

        """
        self.time = duration - self.time
        self.velocity = -self.velocity

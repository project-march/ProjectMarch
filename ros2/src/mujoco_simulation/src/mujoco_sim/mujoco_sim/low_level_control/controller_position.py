"""Author: MVIII."""

from low_level_controller import LowLvlController


class PositionController(LowLvlController):
    """A class which imitates the low-level control of the robot.

    Functions as a PID right now which directly applies control
    to the Mujoco simulation

    Args:
        LowLvlController (Class): Low level controller baseclass
    """
    def __init__(self, node, model, p, d):
        """A class which imitates the low-level control of the robot.

        Functions as a PID right now which directly applies control
        to the Mujoco simulation

        Args:
            node (object ID): the id of the simulation node
            model (Mujoco Struct): Refers to the simulated body in Mujoco
            p (float): Proportional-value of a PD controller
            d (float): Derivative-value of a PD controller
        """
        # Define the amount of controllable joints based on
        # the generalized coordinates generated within Mujoco
        super().__init__(node, model)
        # Define the PD - values
        self.p = p
        self.d = d

    def low_level_update(self, model, data) -> None:
        """The low-level control update callback function.

        This function is called by Mujoco at the start of every simulation step.
        While model isn't used as a variable, the callback function has
        to have this as an argument by Mujoco standards

        Args:
            model (Mujoco struct): Refers to the simulated body in Mujoco
            data (Mujoco struct): Refers to the data struct containing all model data in Mujoco
        """
        dt = self.node.TIME_STEP_MJC
        # update the control inputs based on PID
        try:
            joint_val = self.node.sensor_data_extraction.get_joint_pos()
            for index in range(self.actuator_amount):
                e = self.joint_desired[index] - joint_val[index]
                de_prev = (e - self.e_prev[index]) / dt

                ctrl_input = self.p[index] * e + self.d[index] * de_prev
                data.ctrl[index] += ctrl_input

                self.e_prev[index] = e
        except IndexError:
            pass

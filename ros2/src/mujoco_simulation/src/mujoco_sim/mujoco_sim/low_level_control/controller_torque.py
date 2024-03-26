"""Author: MVIII."""

from low_level_controller import LowLvlController


class TorqueController(LowLvlController):
    """This class implements the low level torque control used by the mujoco simulation.

    When the simulation receives a torque command from the MARCH code, this class is called.
    For the control a PD controller is used.
    """

    def __init__(self, origin, model, p, d):
        """A class which imitates the low-level control of the robot.

        Functions as a PID right now which directly applies control
        to the Mujoco simulation.
        :param origin: (object ID): the id of the simulation node
        :param model: (Mujoco Struct): Refers to the model struct from Mujoco
        :param data: (Mujoco Struct): Refers to the data struct from Mujoco
        :param p: (float): Proportional-value of a PD controller
        :param d: (float): Derivative-value of a PD controller
        """
        super().__init__(origin, model)
        # Define the PD
        self.p = p
        self.d = d

    def low_level_update(self, model, data) -> None:
        """The low-level control update callback function.

        This function is called by Mujoco at the start of every simulation step.
        While model isn't used as a variable, the callback function has
        to have this as an argument by Mujoco standards.
            model (Mujoco struct): Refers to the simulated body in Mujoco
            data (Mujoco struct): Refers to the data struct containing all model data in Mujoco
        """
        dt = self.origin.TIME_STEP_MJC
        # update the control inputs based on PID
        for index in range(self.actuator_amount):
            joint_val = data.qfrc_actuator[index]

            e = self.joint_desired[index] - joint_val
            de_prev = (e - self.e_prev[index]) / dt

            ctrl_input = self.p[index] * e + self.d[index] * de_prev
            data.ctrl[index] += ctrl_input

            self.e_prev[index] = e

    def update_gains(self, p, d, i=None) -> None:
        """Update the PD gains.
        
        :param p: (float list): Proportional-value of a PD controller
        :param d: (float list): Derivative-value of a PD controller
        :param i: (float list): Integration term (not used in this implementation)
        """
        self.p = p
        self.d = d
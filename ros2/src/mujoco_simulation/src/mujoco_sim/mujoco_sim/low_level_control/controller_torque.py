"""Author: MVIII."""

from low_level_controller import LowLvlController


class TorqueController(LowLvlController):
    """This class implements the low level torque control used by the mujoco simulation.

    When the simulation receives a torque command from the MARCH code, this class is called.
    For the control a PD controller is used.
    """

    def __init__(self, origin, model, data, p, d):
        """A class which imitates the low-level control of the robot.

        Functions as a PID right now which directly applies control
        to the Mujoco simulation.
        Args:
        :param origin: (object ID): the id of the simulation node
        :param model: (Mujoco Struct): Refers to the model struct from Mujoco
        :param data: (Mujoco Struct): Refers to the data struct from Mujoco
        :param p: (float): Proportional-value of a PD controller
        :param d: (float): Derivative-value of a PD controller
        """
        # Define the amount of controllable joints based on
        # the generalized coordinates generated within Mujoco
        super().__init__(origin, model, data)
        # Define the PD
        self.p = p
        self.d = d

    def low_level_update(self, model, data):
        """The low-level control update callback function.

        This function is called by Mujoco at the start of every simulation step.
        While model isn't used as a variable, the callback function has
        to have this as an argument by Mujoco standards.
        Args:
            model (Mujoco struct): Refers to the simulated body in Mujoco
            data (Mujoco struct): Refers to the data struct containing all model data in Mujoco
        """
        # update the control inputs based on PID
        for i in range(self.actuator_amount):
            dt = self.origin.TIME_STEP_MJC  # TEMP VARIABLE, REMOVE IN NEXT SPRINT
            joint_val = data.qfrc_actuator[i]
            e = self.joint_ref[i] - joint_val
            de_prev = (e - self.e_prev[i]) / dt

            new_ctrl_input = self.p[i] * e + self.d[i] * de_prev
            self.ctrl[i] = new_ctrl_input
            self.e_prev[i] = e

            data.ctrl[i] += self.ctrl[i]

        # NOTE:CHANGE THE WAY WE UPDATE THE MUJOCO UPDATE LATER

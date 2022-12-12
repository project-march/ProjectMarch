"""Author: MVIII."""

from low_level_controller import LowLvlController


class PositionController(LowLvlController):
    """This class implements the low level position control used by the mujoco simulation.

    When the simulation receives a position command from the MARCH code, this class is called.
    For the control a PD controller is used."""

    def __init__(self, origin, model, data, p, d):
        """A class which imitates the low-level control of the robot.

        Functions as a PID right now which directly applies control 
        to the Mujoco simulation
        Args:
            origin (object ID): the id of the simulation node
            data (Mujoco Struct): Refers to the data struct from Mujoco
            p (float): Proportional-value of a PD controller
            d (float): Derivative-value of a PD controller
        """
        # Define the amount of controllable joints based on
        # the generalized coordinates generated within Mujoco
        super().__init__(origin, model, data)
        # Define the PD
        self.p = p
        self.d = d

    def low_level_update(self, model, data):
        """The low-level control update callback function. This function
        is called by Mujoco at the start of every simulation step.

        While model isn't used as a variable, the callback function has
        to have this as an argument by Mujoco standards
        Args:
            model (Mujoco struct): Refers to the simulated body in Mujoco
            data (Mujoco struct): Refers to the data struct containing all model data in Mujoco
        """

        # update the control inputs based on PID

        for count, act_name in enumerate(self.act_names):
            dt = self.origin.TIME_STEP_MJC
            index = self.joint_to_qpos[model.actuator_trnid[count, 0]]

            joint_val = data.qpos[index]
            try:
                ref_dict = self.joint_ref_dict[act_name]
            except KeyError:
                ref_dict = 0

            e = float(ref_dict) - joint_val
            de_prev = (e - self.e_prev[count]) / dt

            new_ctrl_input = self.p[count] * e + self.d[count] * de_prev
            self.ctrl[count] = new_ctrl_input
            self.e_prev[count] = e
            data.ctrl[count] += self.ctrl[count]

        # NOTE:CHANGE THE WAY WE UPDATE THE MUJOCO UPDATE LATER

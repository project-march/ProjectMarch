from low_level_controller import LowLvlController
import numpy as np


class PositionController(LowLvlController):

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

        # origin.actuator_dict to march to correct array indices.
        # self.joint_ref_dict has the joint name and the ref_pos for that joint
        # To get the correct ref pos loop though keys, get idex from actuator[key_ref_pos] and change that index accoringly.
        # count is
        for count,  joint in enumerate(self.joint_ref_dict):
            dt = 0.001  # TEMP VARIABLE, REMOVE IN NEXT SPRINT
            index = self.joint_to_qpos[model.actuator_trnid[count, 0]]
            joint_val = data.qpos[index]
            e = self.joint_ref_dict[joint] - joint_val
            de_prev = (e - self.e_prev[count]) / dt

            new_ctrl_input = self.p[count] * e + self.d[count] * de_prev
            self.ctrl[count] = new_ctrl_input
            self.e_prev[count] = e

            data.ctrl[count] += self.ctrl[count]

        # NOTE:CHANGE THE WAY WE UPDATE THE MUJOCO UPDATE LATER

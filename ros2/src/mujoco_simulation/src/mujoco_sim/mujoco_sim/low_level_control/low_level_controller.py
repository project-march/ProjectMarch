import numpy as np
from collections import OrderedDict

class LowLvlController():

    def __init__(self, origin, model, data):
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
        self.actuator_amount = model.nu
        self.joint_to_qpos = model.jnt_qposadr
        self.joint_ref = []  # Reference value
        self.e_prev = []  # previous error value for derivative action calculation
        self.ctrl = []  # The control values to be sent to mujoco

        self.act_names = self.get_actuator_names(model)
        name_dict = {}

        for i in range(self.actuator_amount):
            self.joint_ref.append(0)
            name_dict.update({self.act_names[i]: 0})
            self.e_prev.append(0)
            self.ctrl.append(0)
        self.joint_ref_dict = name_dict
        self.origin = origin

    def low_level_update(self, model, data):
        pass

    def get_actuator_names(self, model):
        names = []
        for i in range(model.nu):
            name = ""
            j = model.name_actuatoradr[i]
            while model.names[j] != 0:
                name = name + chr(model.names[j])
                j = j + 1
            names.append(name)
        return names

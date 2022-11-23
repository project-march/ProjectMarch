import numpy as np
import collections


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
        names = []
        for i in range(1, model.njnt):  # Loop starts at 1 because joint 0 is the root joint.
            origin.get_logger().info(str(i))
            name = ""
            j = model.name_jntadr[i]
            while model.names[j] is not 0:
                origin.get_logger().info(str(model.names[j]))
                # ascii_name.append(model.names[j])
                name = name + chr(model.names[j])
                j = j + 1
            names.append(name)
        self.joint_ref_dict = {}

        for i in range(self.actuator_amount):
            self.joint_ref.append(0)
            self.joint_ref_dict.update({names[i]: 0})
            self.e_prev.append(0)
            self.ctrl.append(0)
        self.joint_ref_dict = dict(sorted(self.joint_ref_dict.items()))
        origin.get_logger().info(str(self.joint_ref_dict))
        self.origin = origin

    def low_level_update(self, model, data):
        pass

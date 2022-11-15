import numpy as np

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
        #Define the amount of controllable joints based on
        # the generalized coordinates generated within Mujoco
        self.actuator_amount = model.nu
        self.joint_to_qpos = model.jnt_qposadr
        self.joint_ref = [] #Reference value
        self.e_prev = [] #previous error value for derivative action calculation
        self.ctrl = [] #The control values to be sent to mujoco
        for i in range(self.actuator_amount):
            self.joint_ref.append(0.7)
            self.e_prev.append(0)
            self.ctrl.append(0)
        self.origin = origin
        
    def low_level_update(self, model, data):
        pass


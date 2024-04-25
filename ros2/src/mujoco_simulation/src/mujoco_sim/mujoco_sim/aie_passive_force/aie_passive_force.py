import numpy as np
import mujoco

class AIEPassiveForce:

    def __init__(self, model):
        self.model = model
        self.L_AIE_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_ankle_ie")
        self.R_AIE_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_ankle_ie")

    # CAM v6
    def aie_passive_force(self, x):
        x = - np.rad2deg(x)
        if x > 9.000:
            return 0
        elif x > 6.451:
            return 18.967*x - 122.354
        elif x > 0.510:
            return 0
        elif x > -0.659:
            return 8.720*x - 4.447
        elif x > -3.679:
            return -16.305*x - 20.939
        elif x > -9:
            return 16.469*x + 99.634   
        else:
            return 0

        
    def callback(self, model, data):
        angles = data.qpos
        velocity = data.qvel
        b = 0.95

        data.qfrc_passive[self.L_AIE_id] = 0.5 * self.aie_passive_force(angles[self.L_AIE_id]) - b * velocity[self.L_AIE_id]
        data.qfrc_passive[self.R_AIE_id] = 0.5 * self.aie_passive_force(angles[self.R_AIE_id]) - b * velocity[self.R_AIE_id]
import numpy as np
import mujoco

class AIEPassiveForce:

    def __init__(self, model):
        self.model = model
        self.L_AIE_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "left_ankle_ie")
        self.R_AIE_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "right_ankle_ie")

    def aie_passive_force(self, x):
        x *= 180.0/np.pi # Convert x (deg) to x (radians)
        if x > -0.5926 and x < 1.86:
            return -(35.998*x+10.277+23.361*x**2-19.496*x**3+7.171*x**4+15.106*x**5-26.608*x**6+4.693*x**7+1.651*x**10-8.751*x**9+13.087*x**8)
        elif x >= 1.86 and x < 8.0:
            return -(105.2095-15.94*x)
        else:
            return 0


    def callback(self, model, data):
        angles = data.qpos
        data.qfrc_passive[self.L_AIE_id] = self.aie_passive_force(angles[self.L_AIE_id])
        data.qfrc_passive[self.R_AIE_id] = self.aie_passive_force(angles[self.R_AIE_id])

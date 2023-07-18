"""Author: MVIII."""

import mujoco


class MujocoVisualizer:
    """This class handles the visualisation of mujoco.

    The class calls the api from mujoco to open a window and show the model moving.
    """

    def __init__(self, model, data):
        """This visualizer class handles the window in which Mujoco is rendered.

        We can later decide to turn it on or off by simply turning off the timer.
        Other functionalities, such as camera moving etc can also be added to this
        class.
            model (Mujoco struct): Refers to the simulated body in Mujoco
            data (Mujoco struct): Refers to the data struct containing all model data in Mujoco
        """
        # Initialize the camera and other relevant variables needed for
        # the visualization
        self.cam = mujoco.MjvCamera()
        self.cam.type = 1
        self.opt = mujoco.MjvOption()

        mujoco.glfw.glfw.init()
        self.window = mujoco.glfw.glfw.create_window(1800, 1350, "Sim", None, None)
        mujoco.glfw.glfw.make_context_current(self.window)
        mujoco.glfw.glfw.swap_interval(1)

        mujoco.mjv_defaultCamera(self.cam)
        mujoco.mjv_defaultOption(self.opt)
        # Manually adjust the base camera distance to make the entire model visible
        self.cam.distance = 4.0
        self.cam.azimuth = 135.0
        self.cam.trackbodyid = 1
        self.cam.elevation = -4.5
        self.cam.lookat[2] += 0.7

        self.scene = mujoco.MjvScene(model, maxgeom=10000)
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)
        # Collect the time when the simulation has been started
        self.simstart = data.time

    def update_window(self, model, data):
        """This script is called to update the rendered screen.

        Args:
            model (Mujoco struct): Refers to the simulated body in Mujoco
            data (Mujoco struct): Refers to the data struct containing all model data in Mujoco
        """
        viewport = mujoco.MjrRect(0, 0, 1800, 1350)

        mujoco.mjv_updateScene(model, data, self.opt, None, self.cam, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
        mujoco.mjr_render(viewport, self.scene, self.context)

        mujoco.glfw.glfw.swap_buffers(self.window)
        mujoco.glfw.glfw.poll_events()

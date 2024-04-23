"""Author: MVIII."""


class LowLvlController:
    """A Baseclass which imitates the low-level control of the robot.

    Functions as a PID right now which directly applies control
    to the Mujoco simulation
    """

    def __init__(self, node, model):
        """A class which imitates the low-level control of the robot.

        Args:
            node (id): ID of the main node
            model (Mujoco struct): refers to the simulated body in Mujoco
        """
        self.node = node
        # Define the amount of controllable joints based on
        # the generalized coordinates generated within Mujoco
        self.actuator_amount = model.nu
        self.joint_desired = {}
        self.joint_names = self.node.actuator_names
        self.e_prev = []
        for _i in range(self.actuator_amount):
            self.e_prev.append(0)
        # The map fom sensor data to desired value
        self.sensor_map = []

    def low_level_update(self, model, data) -> None:
        """The low-level control update callback function.

        This function is called by Mujoco at the start of every simulation step.
        While model isn't used as a variable, the callback function has
        to have this as an argument by Mujoco standards
        Args:
            model (Mujoco struct): Refers to the simulated body in Mujoco
            data (Mujoco struct): Refers to the data struct containing all model data in Mujoco
        """

    def map_joint_to_qpos(self, model, actuator_index: int) -> int:
        """Maps the joint indices to the generalized coordinate indices within Mujoco.

        Args:
            model (Mujoco struct): Refers to the simulated body in Mujoco
            actuator_index (int): The joint index

        Returns:
            int: The generalized coordinate index in Mujoco
        """
        return model.jnt_qposadr[model.actuator_trnid[actuator_index, 0]]

    def set_sensor_map(self, sensor_map) -> None:
        """Maps the sensor data to joint references.

        Args:
            sensor_map (integer list): A mapping matrix from sensor data to joint reference
        """
        self.sensor_map = sensor_map

    def update_gains(self, p, d, i) -> None:
        """Updates the PID gains.

        Args:
            p (float list): The proportional gains
            d (float list): The derivative gains
            i (float list): The integral gains
        """
        pass

"""Author: MVIII."""

import mujoco
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_position import PositionController
from controller_torque import TorqueController
from mujoco_interfaces.msg import MujocoDataState
from mujoco_interfaces.msg import MujocoDataSensing
from mujoco_interfaces.msg import MujocoInput
from sensor_msgs.msg import JointState
from mujoco_sim.mujoco_visualize import MujocoVisualizer
from mujoco_sim.sensor_data_extraction import SensorDataExtraction
from queue import Queue, Empty
from rclpy.node import Node
import aie_passive_force


def get_actuator_names(model):
    """This function returns an string array containing the actuator names defined in the mujoco model.

    The names are stored in an array with all other model objet names, with lined addresses.
    To retrieve the names, a loop from the starting address to the terminating char is used.
    """
    names = []
    for i in range(model.nu):
        name = ""
        j = model.name_actuatoradr[i]
        while model.names[j] != 0:
            name = name + chr(model.names[j])
            j = j + 1
        names.append(name)
    return names


def get_controller_data(msg):
    """Get correct joint positions, linked to the joint name, form the incoming message."""
    refs = msg.desired.positions
    joint_names = msg.joint_names
    return dict(zip(joint_names, refs))


def get_data_state_msg(actuator_names, mjc_data):
    """Create a state message from the mujoco data, where the data is linked to the correct joint name."""
    state_msg = MujocoDataState()
    state_msg.names = actuator_names
    state_msg.qpos = mjc_data.qpos.tolist()
    state_msg.qvel = mjc_data.qvel.tolist()
    state_msg.qacc = mjc_data.qacc.tolist()
    state_msg.act = mjc_data.act.tolist()
    # for qpos, qvel, qacc, act in zip(mjc_data.qpos, mjc_data.qvel, mjc_data.qacc, mjc_data.act):
    #     state_msg.qpos.append(qpos)
    #     state_msg.qvel.append(qvel)
    #     state_msg.qacc.append(qacc)
    #     state_msg.act.append(act)
    return state_msg


class MujocoSimNode(Node):
    """This node is the base simulation node.

    In this node the simulation is started, adn the model and data are manipulated.
    Both the reader and writer node communicate with this node to retrieve the data from simulation.
    """

    def __init__(self):
        """This Class is responsible for running the Mujoco simulation.

        A timer is created to update the simulation at a certain rate using
        the sim_step function.
        """
        super().__init__("mujoco_sim")
        self.declare_parameter("model_to_load")
        self.declare_parameter("aie_force")

        self.SIM_TIMESTEP_ROS = 0.008
        self.create_timer(self.SIM_TIMESTEP_ROS, self.sim_update_timer_callback)
        self.time_last_updated = self.get_clock().now()
        # Load in the model and initialize it as a Mujoco object.
        # The model can be found in the robot_description package.
        self.model_name = self.get_parameter("model_to_load")
        self.get_logger().info("Launching Mujoco simulation with robot " + str(self.model_name.value))
        self.file_path = get_package_share_directory("march_description") + "/urdf/march8/" + str(self.model_name.value)
        self.model_string = open(self.file_path, "r").read()
        self.model = mujoco.MjModel.from_xml_path(self.file_path)

        self.data = mujoco.MjData(self.model)

        self.actuator_names = get_actuator_names(self.model)
        self.use_aie_force = self.get_parameter("aie_force")

        if self.use_aie_force.value:
            self.aie_passive_force = aie_passive_force.AIEPassiveForce(self.model)

        # Set timestep options
        self.TIME_STEP_MJC = 0.005
        self.model.opt.timestep = self.TIME_STEP_MJC
        # We need these options to compare mujoco and ros time, so they have the same reference starting point
        self.ros_first_updated = self.get_clock().now()
        # Create a subscriber for the writing-to-mujoco action
        self.writer_subscriber = self.create_subscription(MujocoInput, "mujoco_input", self.writer_callback, 100)
        # Create a publisher for the reading-from-mujoco action
        self.reader_publisher = self.create_publisher(MujocoDataSensing, "mujoco_sensor_output", 10)

        # Initialize the low-level controller
        self.declare_parameters(
            namespace="",
            parameters=[
                ("position.P", None),
                ("position.I", None),
                ("position.D", None),
                ("torque.P", None),
                ("torque.D", None),
            ],
        )

        # Create an instance of that data extractor.
        self.sensor_data_extraction = SensorDataExtraction(self.data.sensordata, self.model.sensor_type,
                                                           self.model.sensor_adr)

        self.set_init_joint_qpos(None)

        joint_val_dict = {}
        joint_val = self.sensor_data_extraction.get_joint_pos()
        for index, name in enumerate(self.actuator_names):
            joint_val_dict[name] = joint_val[index]
        self.get_logger().info(f"Keeping initial joint positions, "
                               f"set desired positions to {joint_val_dict}")

        # This list of controllers contains all active controllers
        self.controller_mode = 0
        self.controller = []
        self.controller.append(
            PositionController(
                self,
                self.model,
                self.get_parameter("position.P").value,
                self.get_parameter("position.D").value,
                self.get_parameter("position.I").value,
                joint_desired=joint_val_dict,
            )
        )
        self.controller.append(
            TorqueController(
                self, self.model, self.get_parameter("torque.P").value, self.get_parameter("torque.D").value
            )
        )
        mujoco.set_mjcb_control(self.controller[self.controller_mode].low_level_update)

        # Create an instance of that data extractor.
        self.sensor_data_extraction = SensorDataExtraction(
            self.data.sensordata, self.model.sensor_type, self.model.sensor_adr
        )

        # Create a queue to store all incoming messages for a correctly timed simulation
        self.msg_queue = Queue()

        # Create the visualizer and visualization timer
        sim_window_fps = 60
        self.visualizer = MujocoVisualizer(self.model, self.data)
        self.create_timer(1 / sim_window_fps, self.sim_visualizer_timer_callback)

        # Create time variables to check when the last trajectory point has been sent. We assume const DT
        self.TIME_STEP_TRAJECTORY = 0.008
        self.trajectory_last_updated = self.get_clock().now()

    def set_init_joint_qpos(self, qpos_init):
        """Set initial qpos to make eo not falling over in sim."""
        if qpos_init is None:
            return

        self.data.qpos[-8:] = qpos_init
        mujoco.mj_step(self.model, self.data)

    def check_for_new_reference_update(self, time_current):
        """This checks if the new trajectory command should be sent.

        The time step is assumed as a constant DT
        Args:
            time_current (Rclpy timee object): The current time
        """
        time_difference = (time_current - self.trajectory_last_updated).to_msg()
        if time_difference.nanosec / 1e9 + time_difference.sec > self.TIME_STEP_TRAJECTORY:
            self.update_trajectory()

    def update_trajectory(self):
        """Updates the trajectory if possible."""
        try:
            msg = self.msg_queue.get_nowait()
            joint_pos = msg
            for j in range(len(self.controller)):
                self.controller[j].joint_desired = joint_pos
            self.trajectory_last_updated = self.get_clock().now()
        except Empty:
            self.get_logger().debug("NO NEW TRAJECTORY FOUND")

    def writer_callback(self, msg):
        """Callback function for the writing service.

        This function enqueues all incoming messages in the message queue.
        With this queue, the sim_update_timer_callback can time the messages correctly in the simulation.
            msg (MujocoControl message): Contains the inputs to be changed
        """
        if msg.reset == 1:
            self.msg_queue = Queue()
        joint_pos_dict = {}
        for i, name in enumerate(msg.trajectory.joint_names):
            joint_pos_dict[name] = msg.trajectory.desired.positions[i]
        self.msg_queue.put(joint_pos_dict)

    def sim_step(self):
        """This function performs the simulation update.

        NOTE: As Mujoco is expected to run faster than ros 2's refresh
        rate limit, modify this so it will repeat multiple time steps
        per rosnode spin event. Also keep track of the time in Mujoco
        vs the time in ROS, so we can update the control inputs on time
        """
        time_current = self.get_clock().now()

        time_shifted = (time_current - self.ros_first_updated).to_msg()

        time_difference_withseconds = time_shifted.nanosec / 1e9 + time_shifted.sec

        if self.use_aie_force.value == 'true':
            while self.data.time <= time_difference_withseconds:
                mujoco.set_mjcb_passive(self.aie_passive_force.callback)
                mujoco.mj_step(self.model, self.data)
        else:
            while self.data.time <= time_difference_withseconds:
                mujoco.mj_step(self.model, self.data)

        self.time_last_updated = self.get_clock().now()

    def sim_update_timer_callback(self):
        """Callback function to perform the simulation step.

        This function is separated from the actual simstep function
        to ensure a nice divide between ROS systems and Mujoco functionality.
        """
        self.check_for_new_reference_update(self.get_clock().now())
        self.publish_sensor_msg()

        self.sim_step()

    def sim_visualizer_timer_callback(self):
        """Callback for the visualization of mujoco.

        :return: None
        """
        self.visualizer.update_window(self.model, self.data)

    def publish_sensor_msg(self):
        """This function creates and publishes the sensor message.

        The state message is published on mujoco_sensor_output.
        NOTE: Since it is still unsure what sensors will be used, this function does not retrieve sensor data yet.
        :return: None
        """
        sensor_msg = MujocoDataSensing()
        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "joint_link"
        state_msg.name = self.actuator_names
        state_msg.position = self.sensor_data_extraction.get_joint_pos()
        state_msg.velocity = self.sensor_data_extraction.get_joint_vel()
        state_msg.effort = self.sensor_data_extraction.get_joint_acc()

        backpack_imu, torso_imu, backpack_position = self.sensor_data_extraction.get_imu_data()
        sensor_msg.joint_state = state_msg
        sensor_msg.backpack_imu = backpack_imu
        sensor_msg.torso_imu = torso_imu
        sensor_msg.backpack_pos = backpack_position

        self.reader_publisher.publish(sensor_msg)


def main(args=None):
    """Main function for life cycle of the node.

    :param args:
    :return:
    """
    rclpy.init(args=args)
    node = MujocoSimNode()
    rclpy.spin(node)
    node.sim_step()
    rclpy.shutdown()
    mujoco.glfw.glfw.terminate()


if __name__ == "__main__":
    main()

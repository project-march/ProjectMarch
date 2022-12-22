"""Author: MVIII."""

import mujoco
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from queue import Queue, Empty

from mujoco_interfaces.msg import MujocoDataState
from mujoco_interfaces.msg import MujocoDataSensing
from mujoco_interfaces.msg import MujocoInput

from mujoco_sim.mujoco_visualize import MujocoVisualizer

from controller_torque import TorqueController
from controller_position import PositionController


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
    for data in mjc_data.qpos:
        state_msg.qpos.append(data)
    for data in mjc_data.qvel:
        state_msg.qvel.append(data)
    for data in mjc_data.qacc:
        state_msg.qacc.append(data)
    for data in mjc_data.act:
        state_msg.act.append(data)
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
        self.declare_parameter('model_toload')

        self.SIM_TIMESTEP_ROS = 0.1
        self.create_timer(self.SIM_TIMESTEP_ROS, self.sim_update_timer_callback)
        self.time_last_updated = self.get_clock().now()
        # Load in the model and initialize it as a Mujoco object.
        # The model can be found in the robot_description package.
        self.model_name = self.get_parameter('model_toload')
        self.file_path = get_package_share_directory('robot_description') + "/urdf/" + str(self.model_name.value)
        self.model_string = open(self.file_path, "r").read()
        self.model = mujoco.MjModel.from_xml_path(self.file_path)

        self.data = mujoco.MjData(self.model)

        self.actuator_names = get_actuator_names(self.model)

        # Set timestep options
        self.TIME_STEP_MJC = 0.0001
        self.model.opt.timestep = self.TIME_STEP_MJC

        # Create a subscriber for the writing-to-mujoco action
        self.writer_subscriber = self.create_subscription(MujocoInput, 'mujoco_input',
                                                          self.writer_callback, 10)

        # Initialize the low-level controller
        self.declare_parameters(
            namespace='',
            parameters=[
                ('position.P', None),
                ('position.D', None),
                ('torque.P', None),
                ('torque.D', None),
            ])
        # This list of controllers contains all active controllers
        self.controller_mode = 0
        self.controller = []
        self.controller.append(PositionController(self, self.model, self.data, self.get_parameter("position.P").value,
                                                  self.get_parameter("position.D").value))
        self.controller.append(TorqueController(self, self.model, self.data, self.get_parameter("torque.P").value,
                                                self.get_parameter("torque.D").value))
        mujoco.set_mjcb_control(self.controller[self.controller_mode].low_level_update)

        # Create a queue to store all incoming messages for a correctly timed simulation
        self.msg_queue = Queue()

        # Create the visualizer and visualization timer
        sim_window_fps = 60
        self.visualizer = MujocoVisualizer(self.model, self.data)
        self.create_timer(1 / sim_window_fps, self.sim_visualizer_timer_callback)

    def writer_callback(self, msg):
        """Callback function for the writing service.

        This function enqueues all incoming messages in hte message queue.
        With this queue, the sim_update_timer_callback can time the messages correctly in the simulation.
            msg (MujocoControl message): Contains the inputs to be changed
        """
        if (msg.reset == 1):
            self.msg_queue.queue.clear()
        self.get_logger().info(str(list(self.msg_queue.queue)))
        self.msg_queue.put(msg.trajectory)

    def sim_step(self):
        """This function performs the simulation update.

        NOTE: As Mujoco is expected to run faster than ros 2's refresh
        rate limit, modify this so it will repeat multiple time steps
        per rosnode spin event. Also keep track of the time in Mujoco
        vs the time in ROS, so we can update the control inputs on time
        """
        time_current = self.get_clock().now()
        time_difference = (time_current - self.time_last_updated).to_msg()
        mj_time_current = self.data.time

        time_difference_withseconds = time_difference.nanosec / 1e9 + time_difference.sec

        while self.data.time - mj_time_current <= time_difference_withseconds:
            mujoco.mj_step(self.model, self.data)

        self.time_last_updated = self.get_clock().now()

    def sim_update_timer_callback(self):
        """Callback function to perform the simulation step.

        This function is separated from the actual simstep function
        to ensure a nice divide between ROS systems and Mujoco functionality.
        """
        # set joint ref to next trajectory point from the queue
        # NOTE: the try catch is needed because at startup the node might run before a trajectory is send,
        # in that case the queue is still empty throwing an exception

        try:
            msg = self.msg_queue.get_nowait()
            joint_pos = get_controller_data(msg)
            for j in range(len(self.controller)):
                self.controller[j].joint_ref_dict = joint_pos
        except Empty:
            pass

        self.publish_state_msg()
        self.publish_sensor_msg()

        self.sim_step()

    def sim_visualizer_timer_callback(self):
        """Callback for the visualization of mujoco.

        :return: None
        """
        self.visualizer.update_window(self.model, self.data)

    def publish_state_msg(self):
        """This function creates and publishes the state message.

        The state message is published on mujoco_state_output.
        The message contains the name, position, velocity, acceleration and act of actuators of the model.
        :return: None
        """
        state_msg = get_data_state_msg(self.actuator_names, self.data)
        publisher = self.create_publisher(
            MujocoDataState, 'mujoco_state_output', 10)
        publisher.publish(state_msg)

    def publish_sensor_msg(self):
        """This function creates and publishes the sensor message.

        The state message is published on mujoco_sensor_output.
        NOTE: Since it is still unsure what sensors will be used, this function does not retrieve sensor data yet.
        :return: None
        """
        sensor_msg = MujocoDataSensing()
        publisher = self.create_publisher(
            MujocoDataSensing, 'mujoco_sensor_output', 10)
        publisher.publish(sensor_msg)


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


if __name__ == '__main__':
    main()

import os
import numpy as np
import mujoco
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from queue import Queue, Empty

from mujoco_interfaces.srv import ReadMujoco
from mujoco_interfaces.msg import MujocoSetControl
from mujoco_interfaces.msg import MujocoDataState
from mujoco_interfaces.msg import MujocoDataSensing
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState

from mujoco_sim.mujoco_visualize import MujocoVisualizer

from controller_torque import TorqueController
from controller_position import PositionController


def get_actuator_names(model):
    names = []
    for i in range(model.nu):
        name = ""
        j = model.name_actuatoradr[i]
        while model.names[j] != 0:
            name = name + chr(model.names[j])
            j = j + 1
        names.append(name)
    return names
class Mujoco_simNode(Node):

    def __init__(self):
        """his Class is responsible for running the Mujoco simulation.
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

        # Create a service so the mujoco_reader node can obtain data from mujoco
        self.serv_read = self.create_service(ReadMujoco, 'read_mujoco', self.read_mujoco)
        # Create a subscriber for the writing-to-mujoco action
        self.writer_subscriber = self.create_subscription(JointTrajectoryControllerState, 'mujoco_input',
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

        # Create a queue to store all incoming messages or correctly timed simulation
        self.msg_queue = Queue()
        self.current_msg = None

        # Create the visualizer and visualization timer
        SIM_WINDOW_FPS = 60
        self.visualizer = MujocoVisualizer(self.model, self.data)
        self.create_timer(1 / SIM_WINDOW_FPS, self.sim_visualizer_timer_callback)

    def writer_callback(self, msg):
        """Callback function for the writing service.
        This function updates the low-level controller reference angles.

        Args:
            msg (MujocoControl message): Contains the inputs to be changed
        """
        self.msg_queue.put(msg)
        # refs = msg.desired.positions
        # joint_names = msg.joint_names
        # joint_pos = dict(zip(joint_names, refs))
        # for j in range(len(self.controller)):
        #     self.controller[j].joint_ref_dict = joint_pos


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

        time_error = abs(time_difference_withseconds - (self.data.time - mj_time_current))
        self.time_last_updated = self.get_clock().now()

    def sim_update_timer_callback(self):
        """Callback function to perform the simulation step.
        This function is separated from the actual simstep function
        to ensure a nice divide between ROS systems and Mujoco functionality.
        """
        try:
            msg = self.msg_queue.get_nowait()
            self.current_msg = msg
            # self.get_logger().info(str(msg) + "\n")
            refs = msg.desired.positions
            joint_names = msg.joint_names
            joint_pos = dict(zip(joint_names, refs))
            for j in range(len(self.controller)):
                self.controller[j].joint_ref_dict = joint_pos
        except Empty:
            pass

        # Publish data, will be put into different functions later on
        state_msg = MujocoDataState()
        state_msg.names = self.actuator_names
        for data in self.data.qpos:
            state_msg.qpos.append(data)

        for data in self.data.qvel:
            state_msg.qvel.append(data)

        for data in self.data.qacc:
            state_msg.qacc.append(data)

        for data in self.data.act:
            state_msg.act.append(data)

        publisher = self.create_publisher(
            MujocoDataState, 'mujoco_state_output', 10)
        publisher.publish(state_msg)

        sensor_msg = MujocoDataSensing()
        publisher = self.create_publisher(
            MujocoDataSensing, 'mujoco_sensor_output', 10)
        publisher.publish(sensor_msg)

        self.sim_step()

    def sim_visualizer_timer_callback(self):

        self.visualizer.update_window(self.model, self.data)
        # self.get_logger().debug(str(self.visualizer.cam))

    def read_mujoco(self, request, response):
        """Server callback function which sends the requested data
        from Mujoco to the client. Only the message type is filled in for the specific request


        Args:
            request (ROS service): ReadMujoco message type(Mujoco_interfaces)
            response (ROS message): Message response depending on the message type

        Returns:
            ROS message: the response to be sent to the client. 
        """
        if request.mujoco_info_type.request == 1:  # If the request was for a sensor state
            response.sensor_state = MujocoDataSensing()
            # NOTE: FOR NOW, PASS NOTHING AS WE DONT HAVE SENSORS YET/
            # WE SHOULD FIGURE OUT SOON WHAT E WANT TO ADD HERE
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Mujoco_simNode()
    rclpy.spin(node)
    node.sim_step()
    rclpy.shutdown()
    mujoco.glfw.glfw.terminate()


if __name__ == '__main__':
    main()

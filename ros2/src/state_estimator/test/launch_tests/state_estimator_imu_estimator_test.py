from lib2to3.pgen2.token import EQUAL
import time
import unittest
import inspect
import yaml

import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from march_shared_msgs.msg import PressureSolesData
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PointStamped

import launch
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.util

import pytest


@pytest.mark.launch_test
def generate_test_description():
    """Specifiy nodes or processes to launch for test
        :param
        :return dut [ros2 node] node to be tested (device under test)
        :return ...,... specifications for launch_testing
    Multiple nodes that are to be tested can be launched"""

    # dut -> device under test is the node to be tested in this example
    config = os.path.join(
        get_package_share_directory("state_estimator"), "config", "state_estimation_setup_params.yaml"
    )

    dut = Node(
        package="state_estimator",
        namespace="",
        executable="state_estimator_node",
        name="state_estimator",
        parameters=[
            config,
        ],
    )
    context = {"dut": dut}

    return (launch.LaunchDescription([dut, launch_testing.actions.ReadyToTest()]), context)


class TestProcessOutput(unittest.TestCase):
    """Details to use this class in the context of launch_testing:
    nodes: https://github.com/ros2/launch_ros
    process: https://github.com/ros2/launch/tree/master/launch_testing"""

    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node("input_output_node")

    def t1_callback(self):
        """Reads a file and publish the data from this file to ros2
        :param -
        :return -
        """
        # Read input data that is send to dut
        msg = JointState()
        msg.name = ["left_ankle"]
        msg.position = [0.3]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.joint_publisher_.publish(msg)

        p_msg = PressureSolesData()
        p_msg.names = ["l_heel_right"]
        p_msg.pressure_values = [1.0]
        self.pressure_publisher_.publish(p_msg)

        i_msg = Imu()
        i_msg.header.stamp = self.node.get_clock().now().to_msg()
        i_msg.header.frame_id = "imu_link"
        i_msg.orientation.x = 0.0
        i_msg.orientation.y = 0.0
        i_msg.orientation.z = 0.0
        i_msg.orientation.w = 1.0
        i_msg.angular_velocity.x = 0.0
        i_msg.angular_velocity.y = 0.0
        i_msg.angular_velocity.z = 0.0
        i_msg.linear_acceleration.x = 0.0
        i_msg.linear_acceleration.y = 0.0
        i_msg.linear_acceleration.z = 0.0
        self.imu_publisher_.publish(i_msg)
        # self.node.get_logger().info('Publishing: ' + str(msg))

    def test_dut_output_invalid_transition(self, dut, proc_output):
        """Listen for a message published by dut and compare message to expected value
        :param
        :return dut [ros2 node] node to be tested (device under test)
        :return proc_output [ActiveIoHandler] data output of dut as shown in terminal (stdout)
        :return -
        """
        # Get current functionname
        frame = inspect.currentframe()
        function_name = inspect.getframeinfo(frame).function

        # Publish data to dut
        self.joint_publisher_ = self.node.create_publisher(JointState, "/joint_states", 10)
        self.pressure_publisher_ = self.node.create_publisher(PressureSolesData, "/march/pressure_sole_data", 10)
        self.imu_publisher_ = self.node.create_publisher(Imu, "/lower_xsens_mti_node", 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.t1_callback)

        # Test sub feet positions
        # Setup for listening to dut messages
        received_data = []
        sub = self.node.create_subscription(
            PointStamped, "/robot_zmp_position", lambda msg: received_data.append(msg.point), 10
        )

        try:
            # Wait until the dut transmits a message over the ROS topic
            end_time = time.time() + 1
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            # test actual output for expected output
            self.assertTrue(len(received_data) != 0)

        finally:
            self.node.destroy_subscription(sub)

    def tearDown(self):
        self.node.destroy_node()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

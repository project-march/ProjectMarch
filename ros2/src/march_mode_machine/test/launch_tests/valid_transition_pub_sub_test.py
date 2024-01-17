from lib2to3.pgen2.token import EQUAL
import time
import unittest
import inspect
import yaml

import os
import ament_index_python

import rclpy
from rclpy.node import Node
from march_shared_msgs.msg import GaitRequest, GaitResponse


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
    dut = Node(
        package="mode_machine",
        executable="mode_machine_node",
        name="mode_machine",
    )

    footstep_gen_node = Node(
        package="footstep_generator", namespace="", executable="footstep_generator_node", name="footstep_generator"
    )

    gait_select_node = Node(
        package="gait_selection", namespace="", executable="gait_selection_node", name="gait_selection"
    )

    context = {"dut": dut}

    return (
        launch.LaunchDescription([dut, footstep_gen_node, gait_select_node, launch_testing.actions.ReadyToTest()]),
        context,
    )


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
        msg = GaitRequest()
        msg.gait_type = 1
        self.publisher_.publish(msg)

    def test_dut_output_valid_transition(self, dut, proc_output):
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
        self.publisher_ = self.node.create_publisher(GaitRequest, "/march/gait_request", 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.t1_callback)

        # expected data for this test is the force unknown state or int num 2.
        expected_data = 1

        # Setup for listening to dut messages
        received_data = []
        sub = self.node.create_subscription(
            GaitResponse, "/march/gait_response", lambda msg: received_data.append(msg.gait_type), 10
        )

        try:
            # Wait until the dut transmits a message over the ROS topic
            end_time = time.time() + 1
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if received_data == []:
                test_data = ""

            else:

                print(f"\n[{function_name}] [INFO] expected_data:\n" + str(expected_data))
                print(f"\n[{function_name}] [INFO] received_data:\n" + str(received_data[0]))
                test_data = received_data[0]

            # test actual output for expected output
            self.assertEqual(test_data, expected_data)

        finally:
            self.node.destroy_subscription(sub)

    def tearDown(self):
        self.node.destroy_node()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

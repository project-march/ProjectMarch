import os
import inspect
import launch
import launch_testing
import launch_testing.actions
import launch_testing.util
import pytest
import rclpy
import time
import unittest
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped


@pytest.mark.launch_test
def generate_test_description():
    """ Specifiy nodes or processes to launch for test
        :param
        :return dut [ros2 node] node to be tested (device under test)
        :return ...,... specifications for launch_testing
    Multiple nodes that are to be tested can be launched"""

    # dut -> device under test is the node to be tested in this example
    config = os.path.join(
        get_package_share_directory('state_estimator'),
        'config',
        'state_estimation_setup_params.yaml'
    )

    dut = Node(
        package='state_estimator',
        namespace='',
        executable='state_estimator_node',
        name='state_estimator',
        parameters=[
            config,
        ]
    )
    context = {'dut': dut}

    return (launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest()]
    ), context
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
        self.node = rclpy.create_node('input_output_node')

    def t1_callback(self):
        """ Reads a file and publish the data from this file to ros2
                :param -
                :return -
            """
        # Read input data that is send to dut
        msg = JointState()
        msg.name = ["left_ankle"]
        msg.position = [0.2]
        msg.velocity = [0.0]
        msg.effort = [0.0]
        self.publisher_.publish(msg)
        # self.node.get_logger().info('Publishing: ' + str(msg))

    def test_dut_output_invalid_transition(self, dut, proc_output):
        """ Listen for a message published by dut and compare message to expected value
                :param
                :return dut [ros2 node] node to be tested (device under test)
                :return proc_output [ActiveIoHandler] data output of dut as shown in terminal (stdout)
                :return -
            """
        # Get current functionname
        frame = inspect.currentframe()
        function_name = inspect.getframeinfo(frame).function

        # Publish data to dut
        self.publisher_ = self.node.create_publisher(JointState, "/joint_states", 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.t1_callback)

        # Setup for listening to dut messages
        received_data = []
        sub = self.node.create_subscription(
            PointStamped,
            '/robot_com_position',
            lambda msg: received_data.append(msg.point),
            10
        )

        try:
            # Wait until the dut transmits a message over the ROS topic
            end_time = time.time() + 10
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

# This file has been created with the help from the following example:
# https://github.com/ros2/launch_ros/blob/master/launch_testing_ros/test/examples/talker_listener_launch_test.py

import os
import unittest

import pytest
import rclpy
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


@pytest.mark.launch_test
def generate_test_description():
    description_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('march_robot_information'),
                     'launch', 'robot_information.launch.py')))
    robot_information_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('march_description'),
                'launch', 'march_description.launch.py')),
        launch_arguments=[('robot_description', 'march4')]
    )
    # The second return value of this tuple can be used to indicate
    # names of the processes such that the stdout of a certain
    # process can be verified.
    # Since we don't need to verify output of processes but just query node
    # parameters this is left as an empty dictionary
    return (LaunchDescription([
        description_node,
        robot_information_node,
        ReadyToTest(),
    ]), {})


class TestRobotInformation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Initialize the ROS context for the test node."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown the ROS context."""
        rclpy.shutdown()

    def setUp(self):
        """Create a ROS node for tests."""
        self.node = rclpy.create_node('test_talker_listener_link')

    def tearDown(self):
        """Destroy the ROS node."""
        self.node.destroy_node()

    def test_get_joint_names(self):
        """Test that all joint names that are set on the robot information
        node parameters are equal to the expected eight joints."""
        client = self.node.create_client(GetParameters,
                                         '/march/robot_information/'
                                         'get_parameters')

        client.wait_for_service()

        while True:
            future = client.call_async(
                GetParameters.Request(names=['joint_names']))

            rclpy.spin_until_future_complete(self.node, future)
            joint_names = future.result().values[0]

            # Only break the loop if the parameter type is set
            if joint_names.type != ParameterType.PARAMETER_NOT_SET:
                break

        self.assertEqual(joint_names.string_array_value,
                         ['left_hip_aa', 'left_hip_fe', 'left_knee',
                          'left_ankle', 'right_hip_aa', 'right_hip_fe',
                          'right_knee', 'right_ankle'])

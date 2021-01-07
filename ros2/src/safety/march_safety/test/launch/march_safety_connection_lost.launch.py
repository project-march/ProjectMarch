import os
import unittest

import pytest
import rclpy
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest

from march_shared_msgs.msg import Alive, Error

from march_safety.test.util import ErrorCounter

SEND_ERRORS_INTERVAL = 3000
INPUT_DEVICE_CONNECTION_TIMEOUT = 3000

@pytest.mark.launch_test
def generate_test_description():
    robot_information_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('march_robot_information_node'),
                     'launch', 'robot_information_node.launch.py')),
        launch_arguments=[('joint_names', '[test_joint1, test_joint2, test_joint3]')]
    )
    safety_node = Node(
        package='march_safety',
        executable='march_safety_node',
        name='safety_node',
        namespace='march',
        output='screen',
        parameters=[
            {'send_errors_interval': SEND_ERRORS_INTERVAL},
            {'input_device_connection_timeout': INPUT_DEVICE_CONNECTION_TIMEOUT}
        ],
    )
    return (LaunchDescription([
        robot_information_node,
        safety_node,
        ReadyToTest(),
    ]), {})


class TestMarchSafetyConnectionLost(unittest.TestCase):
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
        self.node = rclpy.create_node('test_march_safety_connection_lost')
        self.send_errors_interval = SEND_ERRORS_INTERVAL
        self.input_device_connection_timeout = INPUT_DEVICE_CONNECTION_TIMEOUT

    def tearDown(self):
        """Destroy the ROS node."""
        self.node.destroy_node()

    def test_connection_lost(self):
        error_counter = ErrorCounter()
        input_alive_topic = "march/input_device/alive"
        alive_publisher = self.node.create_publisher(msg_type=Alive,
                                                     topic=input_alive_topic,
                                                     qos_profile=0)
        error_topic = "march/error"
        self.node.create_subscription(msg_type=Error,
                                      topic=error_topic,
                                      callback=error_counter.cb,
                                      qos_profile=0)

        check_rate = self.node.create_rate(10)
        while self.node.count_subscribers(input_alive_topic) == 0 or \
                self.node.count_publishers(error_topic) == 0:
            check_rate.sleep()

        self.assertEqual(self.node.count_subscribers(input_alive_topic), 1)
        self.assertEqual(self.node.count_publishers(error_topic), 1)

        alive_publisher.publish(Alive(id="test"))
        rclpy.spin_once(self.node)

        sleep_ms = self.send_errors_interval * 0.9 + self.input_device_connection_timeout
        sleep_rate = self.node.create_rate(1 / sleep_ms)
        sleep_rate.sleep()
        rclpy.spin_once(self.node)

        self.assertEqual(error_counter.count, 1)

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
from test.util import ErrorCounter

INPUT_DEVICE_CONNECTION_TIMEOUT = 500


@pytest.mark.launch_test
def generate_test_description():
    robot_information_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("march_robot_information"),
                "launch",
                "robot_information.launch.py",
            )
        ),
        launch_arguments=[("joint_names", "[test_joint1, test_joint2, test_joint3]")],
    )
    safety_node = Node(
        package="march_safety",
        executable="march_safety_node",
        name="safety_node",
        namespace="march",
        output="screen",
        parameters=[
            {"input_device_connection_timeout": INPUT_DEVICE_CONNECTION_TIMEOUT},
            {"use_sim_time": False},
        ],
    )
    return (
        LaunchDescription(
            [
                robot_information_node,
                safety_node,
                ReadyToTest(),
            ]
        ),
        {},
    )


class TestMarchSafetyConnection(unittest.TestCase):
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
        self.node = rclpy.create_node("test_march_safety_connection")
        self.error_counter = ErrorCounter()

        self.input_device_connection_timeout = INPUT_DEVICE_CONNECTION_TIMEOUT
        self.error_topic = "/march/error"
        self.node.create_subscription(
            msg_type=Error,
            topic=self.error_topic,
            callback=self.error_counter.cb,
            qos_profile=1,
        )

        self.input_alive_topic = "/march/input_device/alive"
        self.alive_publisher = self.node.create_publisher(
            msg_type=Alive, topic=self.input_alive_topic, qos_profile=0
        )

    def tearDown(self):
        """Destroy the ROS node."""
        self.node.destroy_node()

    def test_connection_lost(self):
        self.wait_for_node_startup()
        self.publish_alive()

        # Wait for some time to let error counter increment
        rclpy.spin_once(
            self.node, timeout_sec=(self.input_device_connection_timeout * 3) / 1000
        )

        self.assertEqual(self.error_counter.count, 1)

    def test_connection_not_lost(self):
        self.wait_for_node_startup()
        self.publish_alive()

        # Wait for some time but not enough time to be timed out
        rclpy.spin_once(
            self.node, timeout_sec=(self.input_device_connection_timeout / 2) / 1000
        )

        self.assertEqual(self.error_counter.count, 0)

    def test_connection_never_started(self):
        self.wait_for_node_startup()

        # Wait some time to let input device connection timeout pass
        rclpy.spin_once(
            self.node, timeout_sec=(self.input_device_connection_timeout * 3) / 1000
        )

        self.assertEqual(self.error_counter.count, 0)

    def wait_for_node_startup(self):
        while (
            self.node.count_subscribers(self.input_alive_topic) == 0
            or self.node.count_publishers(self.error_topic) == 0
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertEqual(self.node.count_subscribers(self.input_alive_topic), 1)
        self.assertEqual(self.node.count_publishers(self.error_topic), 1)

    def publish_alive(self):
        self.alive_publisher.publish(
            Alive(id="test", stamp=self.node.get_clock().now().to_msg())
        )

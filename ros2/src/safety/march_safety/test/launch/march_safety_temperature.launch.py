from datetime import datetime, timedelta
import os
import time
import unittest

import pytest
import rclpy
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from march_shared_msgs.msg import Error
from rcl_interfaces.srv import GetParameters, ListParameters
from sensor_msgs.msg import Temperature
from test.util import ErrorCounter

TIMEOUT_DURATION = 3
JOINT_NAMES = ['test_joint1', 'test_joint2', 'test_joint3']
CONFIG_PATH = os.path.join(get_package_share_directory('march_safety'), 'test',
                           'config', 'safety_test.yaml')
THRESHOLD_TYPES = ['warning', 'non_fatal', 'fatal']


@pytest.mark.launch_test
def generate_test_description():
    robot_information_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('march_robot_information'),
                     'launch', 'robot_information.launch.py')),
        launch_arguments=[('joint_names', f"[{','.join(JOINT_NAMES)}]")]
    )
    safety_node = Node(
        package='march_safety',
        executable='march_safety_node',
        name='safety_node',
        namespace='march',
        output='screen',
        parameters=[
            CONFIG_PATH,
            {'use_sim_time': False}
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
        self.node = rclpy.create_node('test_march_safety_temperature')
        self.error_counter = ErrorCounter(self.node)

        self.error_topic = "/march/error"
        self.node.create_subscription(msg_type=Error,
                                      topic=self.error_topic,
                                      callback=self.error_counter.cb,
                                      qos_profile=1)
        self.thresholds = yaml.load(open(CONFIG_PATH).read(), Loader=yaml.FullLoader)[
            'march/safety_node']['ros__parameters']

    def tearDown(self):
        """Destroy the ROS node."""
        self.node.destroy_node()

    def test_all_thresholds_set(self):
        """Test that all expected temperature thresholds are set."""
        parameter_names = []
        for threshold_type in THRESHOLD_TYPES:
            for joint_name in JOINT_NAMES:
                parameter_names.append(f"temperature_thresholds_{threshold_type}.{joint_name}")

        self.assertTrue(self.wait_for_parameter_availability(set(parameter_names + ['use_sim_time', 'default_temperature_threshold', 'send_errors_interval'])),
                        msg=f'Parameters were not set within timeout duration')

        client = self.node.create_client(srv_type=GetParameters,
                                         srv_name='/march/safety_node/get_parameters')
        future = client.call_async(GetParameters.Request(names=parameter_names))
        rclpy.spin_until_future_complete(self.node, future)

        expected_values = [self.get_threshold(*parameter.split('.'))
                           for parameter in parameter_names]
        actual_values = [value.double_value
                         for value in future.result().values]
        self.assertEqual(expected_values, actual_values)

    def test_below_specific_threshold(self):
        joint_name = 'test_joint1'
        publisher = self.create_joint_publisher(joint_name)
        self.wait_for_node_startup(joint_name)

        self.publish_temperature(publisher,
                                 self.get_threshold('temperature_thresholds_non_fatal',
                                                    joint_name) - 1)

        # Wait for some time to let node handle published temperature
        rclpy.spin_once(self.node, timeout_sec=TIMEOUT_DURATION)

        self.assertEqual(self.error_counter.count, 0)

    def test_below_default_threshold(self):
        joint_name = 'test_joint3'
        publisher = self.create_joint_publisher(joint_name)
        self.wait_for_node_startup(joint_name)

        self.publish_temperature(publisher,
                                 self.thresholds['default_temperature_threshold'] - 1)

        # Wait for some time to let node handle published temperature
        rclpy.spin_once(self.node, timeout_sec=TIMEOUT_DURATION)

        self.assertEqual(self.error_counter.count, 0)

    def test_exceed_default_threshold(self):
        joint_name = 'test_joint3'
        publisher = self.create_joint_publisher(joint_name)
        self.wait_for_node_startup(joint_name)

        self.publish_temperature(publisher,
                                 self.thresholds['default_temperature_threshold'] + 1)

        # Wait for some time to let node handle published temperature
        rclpy.spin_once(self.node, timeout_sec=TIMEOUT_DURATION)

        self.assertEqual(self.error_counter.count, 1)

    def test_exceed_default_threshold_multiple_times(self):
        joint_name = 'test_joint3'
        publisher = self.create_joint_publisher(joint_name)
        self.wait_for_node_startup(joint_name)

        times = 3

        for _ in range(times):
            self.publish_temperature(publisher,
                                     self.thresholds['default_temperature_threshold'] + 1)
            # Wait for some time to let node handle published temperature
            rclpy.spin_once(self.node, timeout_sec=TIMEOUT_DURATION)

            # Sleep some extra time to let send_errors_interval pass
            time.sleep(self.thresholds['send_errors_interval'] / 1000)

        self.assertEqual(self.error_counter.count, times)

    def wait_for_parameter_availability(self, parameter_names: set, timeout=TIMEOUT_DURATION) -> bool:
        client = self.node.create_client(srv_type=ListParameters,
                                         srv_name='/march/safety_node/list_parameters')
        client.wait_for_service(TIMEOUT_DURATION)
        start_time = datetime.now()
        while datetime.now() - start_time < timedelta(seconds=timeout):
            future = client.call_async(ListParameters.Request())
            rclpy.spin_until_future_complete(self.node, future)

            if set(future.result().result.names) == parameter_names:
                return True
        return False

    def wait_for_node_startup(self, joint_name):
        while self.node.count_subscribers(self.error_topic) == 0 or \
                self.node.count_publishers(self.get_joint_topic(joint_name)) == 0:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertEqual(self.node.count_subscribers(self.error_topic), 1)
        self.assertEqual(self.node.count_publishers(self.get_joint_topic(joint_name)),
                         1)

    def create_joint_publisher(self, joint_name):
        return self.node.create_publisher(msg_type=Temperature,
                                          topic=self.get_joint_topic(joint_name),
                                          qos_profile=0)

    def get_threshold(self, threshold_type, joint_name):
        if joint_name in self.thresholds[threshold_type]:
            return self.thresholds[threshold_type][joint_name]
        else:
            return self.thresholds['default_temperature_threshold']

    @staticmethod
    def publish_temperature(publisher, temperature):
        msg = Temperature(temperature=temperature)
        publisher.publish(msg)

    @staticmethod
    def get_joint_topic(joint_name):
        return f"march/temperature/{joint_name}"

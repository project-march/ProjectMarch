# import unittest
#
# import pytest
# import rclpy
#
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch_testing.actions import ReadyToTest
# from march_shared_msgs.srv import GetJointNames
#
# JOINT_NAMES = ["joint_1", "joint_2"]
#
#
# @pytest.mark.launch_test
# def generate_test_description():
#     robot_information_node = Node(
#         package="march_robot_information",
#         executable="march_robot_information",
#         output="screen",
#         name="robot_information",
#         namespace="march",
#         parameters=[{"joint_names": JOINT_NAMES}],
#     )
#
#     return (
#         LaunchDescription(
#             [
#                 robot_information_node,
#                 ReadyToTest(),
#             ]
#         ),
#         {},
#     )
#
#
# class TestRobotInformation(unittest.TestCase):
#     """This class provides a launch test for the
#     march_robot_information node."""
#
#     @classmethod
#     def setUpClass(cls):
#         """Initialize the ROS context for the test node."""
#         rclpy.init()
#
#     @classmethod
#     def tearDownClass(cls):
#         """Shutdown the ROS context."""
#         rclpy.shutdown()
#
#     def setUp(self):
#         """Create a ROS node for tests."""
#         self.node = rclpy.create_node("test_robot_information")
#
#     def tearDown(self):
#         """Destroy the ROS node."""
#         self.node.destroy_node()
#
#     def test_get_joint_names(self):
#         """Test that the get_joint_names service return the correct joints."""
#         client = self.node.create_client(
#             GetJointNames, "/march/robot_information/" "get_joint_names"
#         )
#         client.wait_for_service()
#
#         future = client.call_async(GetJointNames.Request())
#         rclpy.spin_until_future_complete(self.node, future)
#         joint_names = future.result().joint_names
#
#         self.assertEqual(joint_names, JOINT_NAMES)

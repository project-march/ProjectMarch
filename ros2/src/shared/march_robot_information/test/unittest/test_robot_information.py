import unittest

import rclpy

from march_robot_information.robot_information_node import RobotInformation
from march_shared_msgs.srv import GetJointNames


class TestGaitSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    def test_create_with_predefined_joint_names(self):
        joint_names = ["one", "two"]
        node = RobotInformation(joint_names=joint_names)
        request = GetJointNames.Request()
        response = GetJointNames.Response(joint_names=[])
        response = node.get_joint_names_cb(request, response)
        self.assertEqual(joint_names, response.joint_names)

import os
import unittest
from ament_index_python import get_package_share_directory

from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.gaits.realsense_gait import RealsenseGait
from march_shared_msgs.msg import GaitParameters

from urdf_parser_py import urdf


TEST_PACKAGE = "march_gait_selection"
TEST_DIRECTORY = "test/resources"


class TestRealsenseGait(unittest.TestCase):
    def setUp(self):
        self.robot = urdf.Robot.from_xml_file(get_package_share_directory("march_description") + "/urdf/march6.urdf")
        self.gait_directory = os.path.join(get_package_share_directory(TEST_PACKAGE), TEST_DIRECTORY)
        self.gait_selection = GaitSelection(gait_package=TEST_PACKAGE, directory=TEST_DIRECTORY, robot=self.robot)
        self.load_realsense_gaits()

    def test_realsense_gaits_loading(self):
        realsense_sit = self.gait_selection.gaits["realsense_sit"]
        realsense_stand = self.gait_selection.gaits["realsense_stand"]

        self.assertTrue(isinstance(realsense_sit, RealsenseGait))
        self.assertTrue(isinstance(realsense_stand, RealsenseGait))

    def load_realsense_gaits(self):
        realsense_sit = self.gait_selection.gaits["realsense_sit"]
        realsense_stand = self.gait_selection.gaits["realsense_stand"]

        self.realsense_sit = realsense_sit
        self.realsense_stand = realsense_stand

    def test_gait_dependencies(self):
        self.assertTrue(self.realsense_stand.gait_name in self.realsense_sit.responsible_for)
        self.assertTrue(self.realsense_sit.gait_name in self.realsense_stand.dependent_on)

    def test_updating_gait_with_responsibilities(self):
        gait_parameters = GaitParameters
        gait_parameters.first_parameter = 0.3
        gait_parameters.second_parameter = 0.5
        gait_parameters.side_step_parameter = 0.3

        self.realsense_sit.update_gaits_from_realsense_call(gait_parameters)
        self.assertEqual(self.realsense_stand.parameters[0], gait_parameters.first_parameter)

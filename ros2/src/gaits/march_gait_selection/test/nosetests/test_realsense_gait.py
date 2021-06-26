import os
import unittest
from ament_index_python import get_package_share_directory

from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.gaits.realsense_gait import RealsenseGait

from urdf_parser_py import urdf


TEST_PACKAGE = "march_gait_selection"
TEST_DIRECTORY = "test/resources"


class TestSetpointsGait(unittest.TestCase):
    def setUp(self):
        self.robot = urdf.Robot.from_xml_file(
            get_package_share_directory("march_description") + "/urdf/march6.urdf"
        )
        self.gait_directory = os.path.join(
            get_package_share_directory(TEST_PACKAGE), TEST_DIRECTORY
        )
        self.gait_selection = GaitSelection(
            gait_package=TEST_PACKAGE, directory=TEST_DIRECTORY, robot=self.robot
        )

    def grab_realsense_gaits(self):
        realsense_sit_gait = self.gait_selection.gaits["realsense_sit"]
        realsense_stand_gait = self.gait_selection.gaits["realsense_stand"]

        self.assertTrue(realsense_sit_gait, RealsenseGait)
        self.assertTrue(realsense_stand_gait, RealsenseGait)

        self.realsense_sit_gait = realsense_sit_gait
        self.realsense_stand_gait = realsense_stand_gait


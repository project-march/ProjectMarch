import os
import unittest
from copy import deepcopy

from ament_index_python import get_package_share_directory
from urdf_parser_py import urdf
import yaml
from march_utility.exceptions.gait_exceptions import (
    SubgaitNameNotFoundError,
    NonValidGaitContentError,
    GaitNameNotFoundError,
)
from march_utility.gait.gait import Gait
from march_utility.gait.subgait import Subgait


class GaitTest(unittest.TestCase):
    def setUp(self):
        self.gait_name = "walk"
        self.robot = urdf.Robot.from_xml_file(get_package_share_directory("march_description") + "/urdf/march6.urdf")
        self.resources_folder = os.path.join(get_package_share_directory("march_utility"), "test", "resources")

        self.default_yaml = os.path.join(self.resources_folder, "default.yaml")
        with open(self.default_yaml, "r") as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)
        self.gait_version_map = default_config["gaits"]

        self.gait = Gait.from_file(self.gait_name, self.resources_folder, self.gait_version_map)

    # Gait.from_file tests
    def test_from_file_valid_path(self):
        self.assertIsInstance(self.gait, Gait)

    def test_from_file_invalid_path(self):
        with self.assertRaises(FileNotFoundError):
            Gait.from_file(
                self.gait_name,
                self.resources_folder + "/gaits",
                self.gait_version_map,
            )

    # __init__ (_validate_trajectory_transition) test
    def test_init_invalid_joint_trajectory_transition(self):
        self.gait.subgaits["left_swing"].joints[0].setpoints[-1].position = 124
        with self.assertRaises(NonValidGaitContentError):
            Gait(self.gait_name, self.gait.subgaits, self.gait.graph)

    # load_subgait tests
    def test_load_existing_subgait(self):
        subgait = Gait.load_subgait(
            self.resources_folder,
            self.gait_name,
            "left_swing",
            self.gait_version_map,
        )
        self.assertIsInstance(subgait, Subgait)

    def test_load_subgait_unexisting_gait_error(self):
        with self.assertRaises(GaitNameNotFoundError):
            Gait.load_subgait(
                self.resources_folder,
                "walk_small",
                "left_swing",
                self.gait_version_map,
            )

    def test_load_subgait_unexisting_subgait_error(self):
        with self.assertRaises(SubgaitNameNotFoundError):
            Gait.load_subgait(
                self.resources_folder,
                self.gait_name,
                "left_open",
                self.gait_version_map,
            )

    def test_load_subgait_unexisting_version_error(self):
        self.gait_version_map["walk"]["right_open"] = "MV_walk_rightopen_non_existing_banana_version"

        with self.assertRaises(FileNotFoundError):
            Gait.load_subgait(
                self.resources_folder,
                self.gait_name,
                "right_open",
                self.gait_version_map,
            )

    def test_set_no_subgait_versions(self):
        self.gait.set_subgait_versions(self.resources_folder, {})

    def test_set_one_new_subgait_version(self):
        subgait_name = "left_close"
        new_version = "MIV_final"
        self.gait.set_subgait_versions(self.resources_folder, {subgait_name: new_version})
        self.assertEqual(new_version, self.gait.subgaits[subgait_name].version)

    def test_set_multiple_subgait_versions(self):
        subgait_name1 = "left_close"
        subgait_name2 = "left_swing"
        new_version = "MIV_final"
        self.gait.set_subgait_versions(
            self.resources_folder,
            {subgait_name1: new_version, subgait_name2: new_version},
        )
        self.assertEqual(new_version, self.gait.subgaits[subgait_name1].version)
        self.assertEqual(new_version, self.gait.subgaits[subgait_name2].version)

    def test_set_version_non_existing_subgait(self):
        with self.assertRaises(SubgaitNameNotFoundError):
            self.gait.set_subgait_versions(self.resources_folder, {"this_subgait_does_not_exist": "1"})

    def test_set_non_existing_version_subgait(self):
        with self.assertRaises(FileNotFoundError):
            self.gait.set_subgait_versions(self.resources_folder, {"left_swing": "1"})

    def test_set_new_subgait_version_invalid_transition(self):
        self.gait.subgaits["right_swing"].joints[0].setpoints[-1].position = 124
        subgait_name = "left_close"
        new_version = "MIV_final"
        with self.assertRaises(NonValidGaitContentError):
            self.gait.set_subgait_versions(self.resources_folder, {subgait_name: new_version})

    def test_set_new_subgait_version_invalid_start(self):
        new_subgaits = deepcopy(self.gait.subgaits)
        new_subgaits["right_open"].joints[0].setpoints[0].position = 124
        with self.assertRaises(NonValidGaitContentError):
            self.gait.set_subgaits(new_subgaits)

    def test_set_new_subgait_version_invalid_final(self):
        new_subgaits = deepcopy(self.gait.subgaits)
        new_subgaits["left_close"].joints[0].setpoints[-1].position = 124
        with self.assertRaises(NonValidGaitContentError):
            self.gait.set_subgaits(new_subgaits)


if __name__ == "__main__":
    unittest.main()

import os
import unittest
from ament_index_python import get_package_share_directory
from march_gait_selection.dynamic_gaits.semi_dynamic_setpoints_gait import (
    SemiDynamicSetpointsGait,
)
from march_gait_selection.gait_selection import GaitSelection
from rclpy.time import Time
from urdf_parser_py import urdf

from march_utility.utilities.duration import Duration

VALID_PACKAGE = "march_gait_selection"
VALID_DIRECTORY = "test/resources"


class TestSemiDynamicGaitSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = urdf.Robot.from_xml_file(
            get_package_share_directory("march_description") + "/urdf/march4.urdf"
        )
        cls.gait_directory = os.path.join(
            get_package_share_directory(VALID_PACKAGE), VALID_DIRECTORY
        )
        cls.gait_selection = GaitSelection(
            gait_package=VALID_PACKAGE, directory=VALID_DIRECTORY, robot=cls.robot
        )

    def setUp(self):
        self.semi_dynamic_gait = SemiDynamicSetpointsGait.from_file(
            "stairs_up",
            self.gait_directory,
            self.robot,
            self.gait_selection.gait_version_map,
        )
        self.semi_dynamic_gait.start(Time(seconds=0))

    def test_can_freeze_begin(self):
        self.assertFalse(self.semi_dynamic_gait.can_freeze)

    def test_can_freeze_after_sec(self):
        self.semi_dynamic_gait.update(Time(seconds=2))
        self.assertTrue(self.semi_dynamic_gait.can_freeze)

    def test_freeze_begin(self):
        self.semi_dynamic_gait.freeze()
        self.assertFalse(self.semi_dynamic_gait._should_freeze)

    def test_freeze_after_sec(self):
        self.semi_dynamic_gait.update(Time(seconds=2))
        self.semi_dynamic_gait.freeze()
        self.assertTrue(self.semi_dynamic_gait._should_freeze)

    def test_subgait_starts_from_0_after_freeze(self):
        self.semi_dynamic_gait.update(Time(seconds=2))
        self.semi_dynamic_gait.freeze(Time(seconds=2))
        self.semi_dynamic_gait.update(Time(seconds=2.5))
        self.assertEqual(self.semi_dynamic_gait._start_time, Time(seconds=2.5))

    def test_freeze_duration(self):
        self.semi_dynamic_gait.update(Time(seconds=2))
        self.semi_dynamic_gait.freeze(Time(seconds=10))
        self.semi_dynamic_gait.update(Time(seconds=3))
        self.semi_dynamic_gait.update(Time(seconds=12.5))
        self.assertEqual(self.semi_dynamic_gait._start_time, Time(seconds=3))

    def test_freeze_duration_done(self):
        self.semi_dynamic_gait.update(Time(seconds=2))
        self.semi_dynamic_gait.freeze(Time(seconds=10))
        self.semi_dynamic_gait.update(Time(seconds=3))
        self.semi_dynamic_gait.update(Time(seconds=14))
        self.assertEqual(self.semi_dynamic_gait._start_time, Time(seconds=14))

    def test_position_after_time_begin(self):
        self.semi_dynamic_gait.update(Time(seconds=1))
        self.assertEqual(
            self.semi_dynamic_gait._position_after_time(Duration(seconds=0)),
            self.gait_selection.positions["stand"]["joints"],
        )

    def test_position_after_time_end(self):
        self.semi_dynamic_gait.update(Time(seconds=2))
        self.assertEqual(
            self.semi_dynamic_gait._position_after_time(Duration(seconds=2.75)),
            {
                "left_ankle": 0.0436,
                "left_hip_aa": 0.0,
                "left_hip_fe": -0.1396,
                "left_knee": 0.3491,
                "right_ankle": 0.0436,
                "right_hip_aa": 0.0,
                "right_hip_fe": 0.3491,
                "right_knee": 0.4363,
            },
        )

    def test_execute_freeze(self):
        self.semi_dynamic_gait.update(Time(seconds=2))
        self.semi_dynamic_gait.freeze()
        self.semi_dynamic_gait.update(Time(seconds=4))
        self.assertEqual(self.semi_dynamic_gait._current_subgait.subgait_name, "freeze")

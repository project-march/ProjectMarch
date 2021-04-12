import os
import unittest
from ament_index_python import get_package_share_directory
from march_gait_selection.state_machine.setpoints_gait import (
    SetpointsGait,
)
from march_gait_selection.gait_selection import GaitSelection
from march_gait_selection.state_machine.gait_interface import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from rclpy.time import Time
from urdf_parser_py import urdf

from march_utility.utilities.duration import Duration

VALID_PACKAGE = "march_gait_selection"
VALID_DIRECTORY = "test/resources"


class TestSetpointsGait(unittest.TestCase):
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
        self.gait = SetpointsGait.from_file(
            "walk",
            self.gait_directory,
            self.robot,
            self.gait_selection.gait_version_map,
        )

    def test_start(self):
        gait_update = self.gait.start(Time(seconds=0))

        self.assertEqual(self.gait.subgait_name, "right_open")
        self.assertEqual(self.gait._start_time, Time(seconds=0))
        self.assertEqual(self.gait._end_time, Time(seconds=1.5))
        self.assertFalse(self.gait._start_is_delayed)

        self.assertEqual(
            gait_update, GaitUpdate.schedule(self.gait._command_from_current_subgait())
        )

    def test_start_delayed(self):
        gait_update = self.gait.start(Time(seconds=0), Duration(seconds=1))

        self.assertEqual(self.gait.subgait_name, "right_open")
        self.assertEqual(self.gait._start_time, Time(seconds=1))
        self.assertEqual(self.gait._end_time, Time(seconds=2.5))
        self.assertTrue(self.gait._start_is_delayed)

        self.assertEqual(
            gait_update,
            GaitUpdate.early_schedule(self.gait._command_from_current_subgait()),
        )

    def test_start_delayed_invalid_duration(self):
        self.gait.start(Time(seconds=0), Duration(seconds=-1))
        self.assertFalse(self.gait._start_is_delayed)

    def test_next_graph_subgait(self):
        self.gait.start(Time(seconds=0))
        self.assertEqual(
            self.gait._next_graph_subgait(), self.gait.subgaits["left_swing"]
        )

    def test_next_graph_subgait_end(self):
        self.gait._current_subgait = self.gait.subgaits["left_close"]
        self.assertIsNone(self.gait._next_graph_subgait())

    def test_next_graph_subgait_stop_valid(self):
        self.gait._current_subgait = self.gait.subgaits["left_swing"]
        self.gait._should_stop = True
        self.assertEqual(
            self.gait._next_graph_subgait(), self.gait.subgaits["right_close"]
        )

    def test_next_graph_subgait_stop_invalid(self):
        self.gait._current_subgait = self.gait.subgaits["right_open"]
        self.gait._should_stop = True
        self.assertEqual(
            self.gait._next_graph_subgait(), self.gait.subgaits["left_swing"]
        )

    def test_stop_true(self):
        self.gait._current_subgait = self.gait.subgaits["left_swing"]
        can_stop = self.gait.stop()
        self.assertTrue(can_stop)
        self.assertTrue(self.gait._should_stop)

    def test_stop_false(self):
        self.gait._current_subgait = self.gait.subgaits["right_open"]
        self.gait._is_transitioning = True
        can_stop = self.gait.stop()
        self.assertFalse(can_stop)
        self.assertFalse(self.gait._should_stop)

    def test_end(self):
        self.gait._current_subgait = self.gait.subgaits["right_open"]
        self.gait.end()
        self.assertIsNone(self.gait._current_subgait)

    def test_update_time_stamps_with_delay(self):
        self.gait._current_time = Time(seconds=3)
        self.gait._update_time_stamps(
            self.gait.subgaits["left_swing"], Duration(seconds=2)
        )
        self.assertEqual(self.gait._start_time, Time(seconds=5))
        self.assertEqual(self.gait._end_time, Time(seconds=6.1))

    def test_update_time_stamps_early(self):
        self.gait._end_time = Time(seconds=2)
        self.gait._scheduled_early = True
        self.gait._update_time_stamps(self.gait.subgaits["right_swing"])
        self.assertEqual(self.gait._start_time, Time(seconds=2))
        self.assertEqual(self.gait._end_time, Time(seconds=3.1))

    def test_update_subgait_done(self):
        self.gait.start(Time(seconds=0))
        gait_update = self.gait.update(Time(seconds=1.8))

        self.assertEqual(self.gait.subgait_name, "left_swing")
        self.assertEqual(self.gait._start_time, Time(seconds=1.8))
        self.assertEqual(self.gait._end_time, Time(seconds=2.9))
        self.assertFalse(self.gait._scheduled_early)

        self.assertEqual(
            gait_update, GaitUpdate.schedule(self.gait._command_from_current_subgait())
        )

    def test_update_subgait_not_done(self):
        self.gait.start(Time(seconds=0))
        gait_update = self.gait.update(Time(seconds=0.6))

        self.assertEqual(self.gait.subgait_name, "right_open")
        self.assertEqual(self.gait._start_time, Time(seconds=0))
        self.assertEqual(self.gait._end_time, Time(seconds=1.5))
        self.assertFalse(self.gait._scheduled_early)

        self.assertEqual(gait_update, GaitUpdate.empty())

    def test_update_subgait_start_delayed_not_started(self):
        self.gait.start(Time(seconds=0), Duration(seconds=3))
        gait_update = self.gait.update(Time(seconds=0.6))

        self.assertEqual(self.gait.subgait_name, "right_open")
        self.assertEqual(self.gait._start_time, Time(seconds=3))
        self.assertEqual(self.gait._end_time, Time(seconds=4.5))
        self.assertTrue(self.gait._start_is_delayed)

        self.assertEqual(gait_update, GaitUpdate.empty())

    def test_update_subgait_start_delayed_did_start(self):
        self.gait.start(Time(seconds=0), Duration(seconds=3))
        gait_update = self.gait.update(Time(seconds=3.5))

        self.assertEqual(self.gait.subgait_name, "right_open")
        self.assertEqual(self.gait._start_time, Time(seconds=3))
        self.assertEqual(self.gait._end_time, Time(seconds=4.5))
        self.assertFalse(self.gait._start_is_delayed)

        self.assertEqual(gait_update, GaitUpdate.subgait_update())

    def test_update_subgait_schedule_early(self):
        self.gait.start(Time(seconds=0))
        gait_update = self.gait.update(Time(seconds=1), Duration(seconds=0.8))

        self.assertEqual(self.gait.subgait_name, "right_open")
        self.assertEqual(self.gait._start_time, Time(seconds=0))
        self.assertEqual(self.gait._end_time, Time(seconds=1.5))
        self.assertTrue(self.gait._scheduled_early)

        self.assertEqual(
            gait_update,
            GaitUpdate.early_schedule(
                TrajectoryCommand.from_subgait(
                    self.gait.subgaits["left_swing"], Time(seconds=1.5)
                )
            ),
        )

        gait_update = self.gait.update(Time(seconds=1.6), Duration(seconds=0.8))
        self.assertEqual(gait_update, GaitUpdate.subgait_update())

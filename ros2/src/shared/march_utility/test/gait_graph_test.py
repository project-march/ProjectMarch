import unittest

import rclpy
from ament_index_python import get_package_share_directory

from urdf_parser_py import urdf

from march_gait_selection.gait_selection import GaitSelection
from march_utility.gait.gait_graph import GaitGraph
from march_utility.gait.edge_position import StaticEdgePosition, UnknownEdgePosition


class TestGaitGraph(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.robot = urdf.Robot.from_xml_file(
            get_package_share_directory("march_description") + "/urdf/march4.urdf"
        )
        cls.stand_position = StaticEdgePosition(
            (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8)
        )

    def setUp(self):
        self.gait_selection = GaitSelection(
            gait_package="march_utility",
            directory="test/resources/gait_graph_gaits",
            robot=self.robot,
            balance=False
        )

    def test_make_named_position(self):
        gait_graph = GaitGraph(self.gait_selection)
        gait_graph._make_named_positions()
        expected = {
            UnknownEdgePosition(): GaitGraph.UNKNOWN,
            self.stand_position: "stand",
        }
        self.assertEqual(expected, gait_graph._named_positions)

    def test_make_transitions(self):
        gait_graph = GaitGraph(self.gait_selection)
        gait_graph._make_named_positions()
        gait_graph._make_transitions()
        expected_idle_transitions = {self.stand_position: {"simple_gait"}}
        expected_gait_transitions = {"simple_gait": self.stand_position}
        self.assertEqual(expected_idle_transitions, gait_graph._idle_transitions)
        self.assertEqual(expected_gait_transitions, gait_graph._gait_transitions)

    def test_make_home_gaits(self):
        gait_graph = GaitGraph(self.gait_selection)
        gait_graph._make_named_positions()
        gait_graph._make_transitions()
        gait_graph._make_home_gaits()
        expected_idle_transitions = {
            self.stand_position: {"simple_gait"},
            self.gait_selection._gaits["home_stand"].starting_position: {"home_stand"},
        }
        self.assertTrue(len(self.gait_selection._gaits) == 2)
        self.assertEqual(expected_idle_transitions, gait_graph._idle_transitions)

    def test_validate_transitions_true(self):
        gait_graph = GaitGraph(self.gait_selection)
        gait_graph._make_named_positions()
        gait_graph._make_transitions()
        gait_graph._make_home_gaits()
        self.assertTrue(gait_graph._validate_from_transitions())
        self.assertTrue(gait_graph._validate_to_transitions())

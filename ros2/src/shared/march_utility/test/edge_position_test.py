from math import pi
import unittest


from march_utility.foot_classes.feet_state import FeetState
from march_utility.foot_classes.foot import Foot
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.side import Side
from march_utility.utilities.utility_functions import (
    get_lengths_robot_for_inverse_kinematics,
)
from march_utility.utilities.vector_3d import Vector3d

from march_utility.gait.edge_position import DynamicEdgePosition, EdgePosition, \
    StaticEdgePosition, UnknownEdgePosition


class EdgePointTest(unittest.TestCase):
    def setUp(self):
        self.values = {"joint1": 0.11, "joint2": -0.25}
        self.values_close = {"joint1": 0.110005, "joint2": -0.25}
        self.values_not_close = {"joint1": 0.12, "joint2": -0.25}

    def test_get_item(self):
        position = EdgePosition(self.values)
        joint_name = "joint1"
        self.assertEqual(self.values[joint_name], position[joint_name])

    def test_eq(self):
        position1 = EdgePosition(self.values)
        position2 = EdgePosition(self.values)
        self.assertEqual(position1, position2)

    def test_eq_is_close(self):
        position1 = EdgePosition(self.values)
        position2 = EdgePosition(self.values_close)
        self.assertEqual(position1, position2)

    def test_eq_not_is_close(self):
        position1 = EdgePosition(self.values)
        position2 = EdgePosition(self.values_not_close)
        self.assertNotEqual(position1, position2)

    def test_is_compatible_parent(self):
        position1 = EdgePosition(self.values)
        position2 = EdgePosition(self.values)
        self.assertTrue(position1.is_compatible(position2))

    def test_is_compatible_static(self):
        position1 = StaticEdgePosition(self.values)
        position2 = StaticEdgePosition(self.values)
        self.assertTrue(position1.is_compatible(position2))

    def test_is_compatible_dynamic(self):
        position1 = DynamicEdgePosition(self.values)
        position2 = DynamicEdgePosition(self.values)
        self.assertTrue(position1.is_compatible(position2))

    def test_is_compatible_static_dynamic(self):
        position1 = StaticEdgePosition(self.values)
        position2 = DynamicEdgePosition(self.values)
        self.assertFalse(position1.is_compatible(position2))

    def test_is_compatible_dynamic_static(self):
        position1 = DynamicEdgePosition(self.values)
        position2 = StaticEdgePosition(self.values)
        self.assertFalse(position1.is_compatible(position2))

    def test_unknown(self):
        position = UnknownEdgePosition()
        self.assertIsNone(position.values)

    def test_is_compatible_unknown(self):
        position1 = UnknownEdgePosition()
        position2 = UnknownEdgePosition()
        self.assertTrue(position1.is_compatible(position2))
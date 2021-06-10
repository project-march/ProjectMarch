import unittest

from march_utility.gait.edge_position import (
    EdgePosition,
    UnknownEdgePosition,
)


class EdgePointTest(unittest.TestCase):
    def setUp(self):
        self.values = {"joint1": 0.11, "joint2": -0.25}
        self.values_close = {"joint1": 0.110005, "joint2": -0.25}
        self.values_not_close = {"joint1": 0.12, "joint2": -0.25}

    def test_get_item(self):
        position = EdgePosition(self.values)
        self.assertEqual(self.values["joint1"], position[0])

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

    def test_unknown(self):
        position = UnknownEdgePosition()
        self.assertEqual(position.values, ())

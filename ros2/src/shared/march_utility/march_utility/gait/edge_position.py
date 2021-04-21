import math
from typing import Dict


class EdgePosition:
    ALLOWED_ERROR_ENDPOINTS = 0.001
    JointDictionary = Dict[str, float]

    def __init__(self, values: JointDictionary):
        self.values = values

    def __getitem__(self, item) -> float:
        return self.values[item]

    def __eq__(self, other):
        if isinstance(other, EdgePosition):
           return all(math.isclose(self[joint], other[joint], rel_tol=0, abs_tol=self.ALLOWED_ERROR_ENDPOINTS)
                for joint in self.values.keys())

    def is_compatible(self, other):
        return self == other


class StaticEdgePosition(EdgePosition):
    def __init__(self, values):
        super().__init__(values)

    def __eq__(self, other):
        if isinstance(other, StaticEdgePosition):
            return super().__eq__(other)
        return False

    def is_compatible(self, other):
        if isinstance(other, StaticEdgePosition):
            return super().is_compatible(other)
        return False


class DynamicEdgePosition(EdgePosition):
    def __init__(self, values):
        super().__init__(values)

    def __eq__(self, other):
        if isinstance(other, DynamicEdgePosition):
            return super().__eq__(other)
        return False

    def is_compatible(self, other):
        if isinstance(other, DynamicEdgePosition):
            return True
        return False


class UnknownEdgePosition(EdgePosition):
    def __init__(self):
        super().__init__(None)

    def __eq__(self, other):
        return isinstance(other, UnknownEdgePosition)

    def is_compatible(self, other):
        return self == other
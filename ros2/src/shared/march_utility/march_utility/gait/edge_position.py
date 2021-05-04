import math
from typing import Dict, Union


class EdgePosition:
    ALLOWED_ERROR_ENDPOINTS = 0.001

    JointDictionary = Dict[str, float]

    def __init__(self, values: Union[tuple, list, JointDictionary]):
        if isinstance(values, dict):
            self.values = tuple(value for _, value in sorted(values.items()))
        elif isinstance(values, list):
            self.values = tuple(values)
        else:
            self.values = values

    def __getitem__(self, item) -> float:
        return self.values[item]

    def __eq__(self, other):
        if isinstance(other, EdgePosition):
            return all(
                math.isclose(
                    value,
                    other.values[i],
                    rel_tol=0,
                    abs_tol=self.ALLOWED_ERROR_ENDPOINTS,
                )
                for i, value in enumerate(self.values)
            )

    def __str__(self):
        return str(self.values)

    def __hash__(self):
        return hash(self.values)

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

    def __hash__(self):
        return super().__hash__()


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

    def __hash__(self):
        return super().__hash__()


class UnknownEdgePosition(EdgePosition):
    def __init__(self):
        super().__init__({})

    def __eq__(self, other):
        return isinstance(other, UnknownEdgePosition)

    def is_compatible(self, other):
        return self == other

    def __hash__(self):
        return super().__hash__()

import math
from typing import Dict, Union


class EdgePosition:
    """
    A class that is used to denote the starting and final positions of gaits. This is
    used both when interpolating gaits, to verify that static end positions don't
    accidentally change and to determine the possible transitions on startup.
    """

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


class StaticEdgePosition(EdgePosition):
    """
    EdgePosition for when no changes can be done during runtime. These are best for
    determining transitions beforehand and will also be named in the default.yaml.
    """

    def __init__(self, values):
        super().__init__(values)

    def __eq__(self, other):
        if isinstance(other, StaticEdgePosition):
            return super().__eq__(other)
        return False

    def __hash__(self):
        return super().__hash__()

    def __str__(self):
        return f"Static: {self.values}"


class DynamicEdgePosition(EdgePosition):
    """
    EdgePosition for gaits that allow changing at runtime, for example based on the
    camera images. These transitions are checked extra when requesting the gaits.
    """

    def __init__(self, values):
        super().__init__(values)

    def __eq__(self, other):
        if isinstance(other, DynamicEdgePosition):
            return super().__eq__(other)
        return False

    def __hash__(self):
        return super().__hash__()

    def __str__(self):
        return f"Dynamic: {self.values}"


class UnknownEdgePosition(EdgePosition):
    """
    EdgePosition for gaits that can start from an unknown idle position.
    """

    def __init__(self):
        super().__init__({})

    def __eq__(self, other):
        return isinstance(other, UnknownEdgePosition)

    def __hash__(self):
        return super().__hash__()

"""Author: Bas Volkers, MVI."""
import math
from typing import Dict, Union


class EdgePosition:
    """A class that is used to denote the starting and final positions of gaits.

    This is used both when interpolating gaits:
        * To verify that static end positions don't accidentally change.
        * To determine the possible transitions on startup.
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
        """float. Returns the value for the given item key."""
        return self.values[item]

    def __eq__(self, other) -> bool:
        """bool. Checks if the other object is also an EdgePosition and if it is close enough to this EdgePosition."""
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
        else:
            return False

    def __str__(self):
        """str. Returns a dictionary string representation of the values."""
        return str(self.values)

    def __hash__(self):
        """Returns the hash value of the value dictionary."""
        return hash(self.values)


class StaticEdgePosition(EdgePosition):
    """EdgePosition for when no changes can be done during runtime.

    These are best for determining transitions beforehand and will also be named in the default.yaml.
    """

    def __eq__(self, other):
        """bool. Checks if the `other` is also an StaticEdgePosition and if it is close enough to this position."""
        if isinstance(other, StaticEdgePosition):
            return super().__eq__(other)
        return False

    def __str__(self):
        """str. Returns a dictionary string representation of the values, with 'Static' added in front."""
        return f"Static: {self.values}"

    def __hash__(self):
        """Returns the hash value of the value dictionary."""
        return super().__hash__()


class DynamicEdgePosition(EdgePosition):
    """EdgePosition for gaits that allow changing at runtime.

    An example of such position are camera images. These transitions are checked extra when requesting the gaits.
    """

    def __eq__(self, other):
        """bool. Checks if the `other` is also an DynamicEdgePosition and if it is close enough to this position."""
        if isinstance(other, DynamicEdgePosition):
            return super().__eq__(other)
        return False

    def __str__(self):
        """str. Returns a dictionary string representation of the values, with 'Dynamic' added in front."""
        return f"Dynamic: {self.values}"

    def __hash__(self):
        """Returns the hash value of the value dictionary."""
        return super().__hash__()


class UnknownEdgePosition(EdgePosition):
    """EdgePosition for gaits that can start from an unknown idle position."""

    def __init__(self):
        super().__init__({})

    def __eq__(self, other):
        """bool. Checks if the `other` is also an UnknownEdgePosition."""
        return isinstance(other, UnknownEdgePosition)

    def __hash__(self):
        """Returns the hash value of the value dictionary."""
        return super().__hash__()

"""Author: Katja Schmal, MVI."""
from enum import Enum
from march_utility.exceptions.gait_exceptions import (
    UnknownDimensionsError,
    WrongRealSenseConfigurationError,
)


class InterpolationDimensions(Enum):
    """Enum for all supported dimensions, of the interpolation code.

    This is currently one 1D and 2D, but might be extended.
    """

    ONE_DIM = 1
    TWO_DIM = 2

    @classmethod
    def from_integer(cls, dimensions: int):
        """Method to create the enum based on the integer value."""
        if dimensions == 1:
            return cls.ONE_DIM
        elif dimensions == 2:
            return cls.TWO_DIM
        else:
            raise WrongRealSenseConfigurationError(f"The dimensions value is not `1` or `2`, but {dimensions}")


def amount_of_subgaits(dim: InterpolationDimensions) -> int:
    """Returns the amount of subgaits needed with a given dimension.

    Raises:
        UnknownDimensionsError: If the given dimensions is not supported yet,
            see InterpolationDimensions enum for supported dimensions.
    """
    if dim == InterpolationDimensions.ONE_DIM:
        return 2
    elif dim == InterpolationDimensions.TWO_DIM:
        return 4
    else:
        raise UnknownDimensionsError(dim)


def amount_of_parameters(dim: InterpolationDimensions) -> int:
    """Returns the amount of subgaits needed with a given dimension.

    Raises:
        UnknownDimensionsError: If the given dimensions is not supported yet,
            see InterpolationDimensions enum for supported dimensions.
    """
    if dim == InterpolationDimensions.ONE_DIM:
        return 1
    elif dim == InterpolationDimensions.TWO_DIM:
        return 2
    else:
        raise UnknownDimensionsError(dim)

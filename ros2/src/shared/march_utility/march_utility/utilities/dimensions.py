from enum import Enum
from march_utility.exceptions.gait_exceptions import (
    UnknownDimensionsError,
    WrongRealSenseConfigurationError,
)


# This enumeration contains all amount of dimensions that are supported by the
# interpolation code, this is currently one 1D and 2D, but might be extended


class InterpolationDimensions(Enum):
    ONE_DIM = 1
    TWO_DIM = 2

    @classmethod
    def from_integer(cls, dimensions: int):
        if dimensions == 1:
            return cls.ONE_DIM
        elif dimensions == 2:
            return cls.TWO_DIM
        else:
            raise WrongRealSenseConfigurationError(
                f"The dimensions value is not `1` " f"or `2`, but {dimensions}"
            )


def amount_of_subgaits(dim: InterpolationDimensions):
    if dim == InterpolationDimensions.ONE_DIM:
        return 2
    elif dim == InterpolationDimensions.TWO_DIM:
        return 4
    else:
        raise UnknownDimensionsError(dim)


def amount_of_parameters(dim: InterpolationDimensions):
    if dim == InterpolationDimensions.ONE_DIM:
        return 1
    elif dim == InterpolationDimensions.TWO_DIM:
        return 2
    else:
        raise UnknownDimensionsError(dim)

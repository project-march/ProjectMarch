class GaitError(Exception):
    def __init__(self, msg: str = None):
        """
        Initialize a basic gait error exception.
        :param msg: The message to display.
        """
        if msg is None:
            msg = "An error occurred with a gait module."
        super(GaitError, self).__init__(msg)


class GaitNameNotFoundError(GaitError):
    def __init__(self, gait_name: str, msg: str = None):
        """Class to raise an error when given gait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Could not find gait name: {gait} in map.".format(gait=gait_name)

        super(GaitNameNotFoundError, self).__init__(msg)


class SubgaitNameNotFoundError(GaitError):
    def __init__(self, subgait_name: str, gait_name: str, msg: str = None):
        """Class to raise an error when given subgait name does not exists .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Could not find subgait name {subgait} of gait {gait} in map.".format(
                subgait=subgait_name, gait=gait_name
            )

        super(SubgaitNameNotFoundError, self).__init__(msg)


class NonValidGaitContentError(GaitError):
    def __init__(self, gait_name: str = None, msg: str = None):
        """Class to raise an error when given gait has incorrect content .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "The given gait: {gn} has incorrect information".format(gn=gait_name)

        super(NonValidGaitContentError, self).__init__(msg)


class SubgaitGraphError(GaitError):
    def __init__(self, msg: str):
        super(SubgaitGraphError, self).__init__(msg)


class TransitionError(Exception):
    def __init__(self, msg: str = None):
        """Class to raise an error when transition between two subgaits has an error .

        :param msg:
            The message to display.
        """
        if msg is None:
            msg = "Subgaits can not transition"

        super(TransitionError, self).__init__(msg)


class SubgaitInterpolationError(Exception):
    def __init__(self, msg: str = None):
        """Class to raise an error when it was not possible to interpolate between subgaits."""
        if msg is None:
            msg = "An error occurred while trying to merge two subgaits."

        super(SubgaitInterpolationError, self).__init__(msg)


class UnknownDimensionsError(Exception):
    def __init__(self, dimensions):
        msg = (
            f"Unknown amount of dimensions, should be from InterpolationDimensions "
            f"enum, but was: {dimensions}"
        )
        super(UnknownDimensionsError, self).__init__(msg)


class WrongRealSenseConfigurationError(Exception):
    def __init__(self, msg: str = None):
        """Class to raise an error when there was a mistake in the
        realsense_gaits.yaml."""
        if msg is None:
            msg = "An error occurred while trying to read out the realsense config."

        super(WrongRealSenseConfigurationError, self).__init__(msg)


class PositionSoftLimitError(Exception):
    def __init__(
        self, joint_name: str, position: float, lower_limit: float, upper_limit: float
    ):
        """Class to raise an error when joint trajectory will be outside of
        position soft limits"""
        self.joint_name = joint_name
        self.position = position
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

        msg = (
            f"{joint_name} will be outside its soft limits. "
            f"position: {position}, soft limits: "
            f"[{lower_limit}, {upper_limit}]."
        )

        super(PositionSoftLimitError, self).__init__(msg)


class VelocitySoftLimitError(Exception):
    def __init__(self, joint_name: str, velocity: float, limit: float):
        """Class to raise an error when joint trajectory will be outside of
        velocity soft limits"""
        self.joint_name = joint_name
        self.velocity = velocity
        self.limit = limit

        msg = (
            f"{joint_name} will be outside of velocity limits, "
            f"velocity: {velocity}, velocity limit: {limit}."
        )

        super(VelocitySoftLimitError, self).__init__(msg)

class GaitError(Exception):
    """Base class for exceptions in gait modules.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "An error occurred with a gait module."
        super(GaitError, self).__init__(msg)


class GaitNameNotFoundError(GaitError):
    """Class to raise an error when given gait name does not exists .

    Args:
        gait_name (str): The name of the gait that could not be found.
        msg (str, optional): The error message to display.
    """

    def __init__(self, gait_name, msg=None):
        if msg is None:
            msg = "Could not find gait name: {gait} in map.".format(gait=gait_name)

        super(GaitNameNotFoundError, self).__init__(msg)


class SubgaitNameNotFoundError(GaitError):
    """Class to raise an error when given subgait name does not exists .

    Args:
        subgait_name (str): The name of the subgait that is not recognized within the gait.
        gait_name (str): The name of the gait that could not be found.
        msg (str, optional): The error message to display.
    """

    def __init__(self, subgait_name, gait_name, msg=None):
        if msg is None:
            msg = "Could not find subgait name {subgait} of gait {gait} in map.".format(
                subgait=subgait_name, gait=gait_name
            )

        super(SubgaitNameNotFoundError, self).__init__(msg)


class NonValidGaitContentError(GaitError):
    """Class to raise an error when given gait has incorrect content .

    Args:
        gait_name (str): The name of the gait that could not be found.
        msg (str, optional): The error message to display.
    """

    def __init__(self, gait_name=None, msg=None):
        if msg is None:
            msg = "The given gait: {gn} has incorrect information".format(gn=gait_name)

        super(NonValidGaitContentError, self).__init__(msg)


class SubgaitGraphError(GaitError):
    """Todo: Add docstring."""

    def __init__(self, msg):
        super(SubgaitGraphError, self).__init__(msg)


class TransitionError(GaitError):
    """Class to raise an error when transition between two subgaits has an error.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "Subgaits can not transition"

        super(TransitionError, self).__init__(msg)


class SubgaitInterpolationError(GaitError):
    """Class to raise an error when it was not possible to interpolate between subgaits.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "An error occurred while trying to merge two subgaits."

        super(SubgaitInterpolationError, self).__init__(msg)

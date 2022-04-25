# TODO(Thijs): Update this class to the Python 3 builtin FileNotFoundError.
# Python 2 did not have this.
# https://gitlab.com/project-march/march/-/issues/663


class FileNotFoundError(Exception):  # noqa: A001
    """Class to raise an error when a file cannot be found.

    Args:
        file_path: The file path which is not found by os.path.isfile().
        msg (str, optional): The error message to display.
    """

    def __init__(self, file_path, msg=None):

        if msg is None:
            msg = "File path: {fp} could not be found.".format(fp=file_path)

        super(FileNotFoundError, self).__init__(msg)


class PackageNotFoundError(Exception):
    """Class to raise an error when a ros1 package cannot be found.

    Args:
        package_name: The package name which is not found by rospkg.RosPack().get_path()
        msg (,optional) a custom error message to return.
    """

    def __init__(self, package_name, msg=None):
        if msg is None:
            msg = "Package: {fp} could not be found.".format(fp=package_name)

        super(PackageNotFoundError, self).__init__(msg)


class MsgTypeError(Exception):
    """Class to raise an error when an non msg type is added to a message.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "A non msg type (defined in shared resources) was added to a ROS-message"

        super(MsgTypeError, self).__init__(msg)


class SideSpecificationError(Exception):
    """Class to raise an error when a foot ('right' or 'left') has to be specified but this did not happen.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, foot, msg=None):
        if msg is None:
            msg = "An incorrect side was supplied. Must be a either Side.left or Side.right, but was {foot}.".format(
                foot=foot
            )

        super(SideSpecificationError, self).__init__(msg)


class IncorrectCoordinateError(Exception):
    """Class to raise an error when the coordinates of a position are incorrect.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "The keys of a position or velocity dictionary should be ['x', 'y', 'z'], but were different."

        super(IncorrectCoordinateError, self).__init__(msg)


class WeightedAverageError(Exception):
    """Class to raise an error when a weighted average cannot be computed.

    Args:
        msg (str, optional): The error message to display.
    """

    def __init__(self, msg=None):
        if msg is None:
            msg = "The calculation of the weighted average cannot be executed safely."

        super(WeightedAverageError, self).__init__(msg)

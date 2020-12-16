import ast

import rospy
from std_srvs.srv import Trigger

from march_shared_msgs.srv import SetGaitVersion

from .gait_selection_errors import GaitServiceError, InvalidResponseError


class GaitSelectionController(object):
    def __init__(self):
        """Base class to communicate with the gait selection node."""
        self._get_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)
        self._get_directory_structure = rospy.ServiceProxy('/march/gait_selection/get_directory_structure', Trigger)

        self._set_gait_version = rospy.ServiceProxy('/march/gait_selection/set_gait_version', SetGaitVersion)
        self._set_default_versions = rospy.ServiceProxy('/march/gait_selection/update_default_versions', Trigger)

    def get_version_map(self):
        """Get the gait version map used in the gait selection node."""
        try:
            return dict(ast.literal_eval(self._get_version_map().message))
        except ValueError:
            raise InvalidResponseError
        except rospy.ServiceException:
            raise GaitServiceError

    def get_directory_structure(self):
        """Get the gait directory of the selected gait_directory in the gait selection node."""
        try:
            return dict(ast.literal_eval(self._get_directory_structure().message))
        except ValueError:
            raise InvalidResponseError
        except rospy.ServiceException:
            raise GaitServiceError

    def set_gait_version(self, gait_name, subgait_names, versions):
        """Set a new gait version map to use in the gait selection node.

        :param str gait_name: The name of the gait
        :param list(str) subgait_names: Names of subgaits of which to change the version
        :param list(str) versions: Names of the versions
        """
        try:
            result = self._set_gait_version(gait_name, subgait_names, versions)
            return result.success, result.message
        except rospy.ServiceException:
            raise GaitServiceError

    def set_default_versions(self):
        """Save the current gait version map in the gait selection node as a default."""
        try:
            result = self._set_default_versions()
            return result.success, result.message
        except rospy.ServiceException:
            raise GaitServiceError

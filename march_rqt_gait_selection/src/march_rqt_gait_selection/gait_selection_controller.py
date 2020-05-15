
import ast
import sys

import rospy

from march_shared_resources.srv import StringTrigger, Trigger


class GaitSelectionController(object):
    def __init__(self):
        """Base class to communicate with the gait selection node."""
        try:
            rospy.wait_for_service('/march/gait_selection/get_version_map', 3)
            rospy.wait_for_service('/march/gait_selection/get_directory_structure', 3)
            rospy.wait_for_service('/march/gait_selection/set_version_map', 3)
            rospy.wait_for_service('/march/gait_selection/update_default_versions', 3)
        except rospy.ROSException:
            rospy.logerr('Shutting down march_rqt_gait_selection, could not connect to march_gait_selection.')
            rospy.signal_shutdown('Shutting down march_rqt_gait_selection, could not connect to march_gait_selection.')
            sys.exit(0)

        self._get_version_map = rospy.ServiceProxy('/march/gait_selection/get_version_map', Trigger)
        self._get_directory_structure = rospy.ServiceProxy('/march/gait_selection/get_directory_structure', Trigger)

        self._set_version_map = rospy.ServiceProxy('/march/gait_selection/set_version_map', StringTrigger)
        self._set_default_versions = rospy.ServiceProxy('/march/gait_selection/update_default_versions', Trigger)

    def get_version_map(self):
        """Get the gait version map used in the gait selection node."""
        try:
            return dict(ast.literal_eval(self._get_version_map().message))
        except ValueError:
            return None

    def get_directory_structure(self):
        """Get the gait directory of the selected gait_directory in the gait selection node."""
        try:
            return dict(ast.literal_eval(self._get_directory_structure().message))
        except ValueError:
            return None

    def set_version_map(self, gait_version_map):
        """Set a new gait version map to use in the gait selection node.

        :param gait_version_map:
            The new gait version map as dictionary to parse to the gait selection node
        """
        result = self._set_version_map(str(gait_version_map))
        if result.success:
            return True
        else:
            return False

    def set_default_versions(self):
        """Save the current gait version map in the gait selection node as a default."""
        result = self._set_default_versions()
        if result.success:
            return True
        else:
            return False

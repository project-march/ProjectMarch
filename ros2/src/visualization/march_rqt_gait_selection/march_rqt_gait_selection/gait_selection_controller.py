import ast
import rclpy
from std_srvs.srv import Trigger
from march_shared_msgs.srv import SetGaitVersion
from .gait_selection_errors import GaitServiceError, InvalidResponseError


class GaitSelectionController(object):
    def __init__(self, node):
        """Base class to communicate with the gait selection node."""
        self._node = node
        self._get_version_map = node.create_client(
            srv_type=Trigger, srv_name='/march/gait_selection/get_version_map')
        self._get_directory_structure = node.create_client(
            srv_type=Trigger, srv_name='/march/gait_selection/get_directory_structure')

        self._set_gait_version = node.create_client(
            srv_type=SetGaitVersion, srv_name='/march/gait_selection/set_gait_version')
        self._set_default_versions = node.create_client(
            srv_type=Trigger, srv_name='/march/gait_selection/update_default_versions')

    def get_version_map(self):
        """Get the gait version map used in the gait selection node."""
        try:
            while not self._get_version_map.wait_for_service(timeout_sec=2):
                self._node.get_logger().warn(
                    "Waiting for get_version_map service to be available, is gait selection running?")
            return dict(ast.literal_eval(self._get_version_map.call(Trigger.Request()).message))
        except ValueError:
            raise InvalidResponseError
        # except rclpy.ServiceException:
        #     raise GaitServiceError

    def get_directory_structure(self):
        """Get the gait directory of the selected gait_directory in the gait selection node."""
        try:
            while not self._get_version_map.wait_for_service(timeout_sec=2):
                self._node.get_logger().warn(
                    "Waiting for get_directory_structure service to be available, is gait selection running?")
            return dict(ast.literal_eval(self._get_directory_structure.call(Trigger.Request()).message))
        except ValueError:
            raise InvalidResponseError
        # except rospy.ServiceException:
        #     raise GaitServiceError

    def set_gait_version(self, gait_name, subgait_names, versions):
        """Set a new gait version map to use in the gait selection node.

        :param str gait_name: The name of the gait
        :param list(str) subgait_names: Names of subgaits of which to change the version
        :param list(str) versions: Names of the versions
        """
        # try:
        while not self._get_version_map.wait_for_service(timeout_sec=2):
            self._node.get_logger().warn("Waiting for set_gait_version service to be available, is gait selection running?")
        result = self._set_gait_version.call(
            SetGaitVersion.Request(gait=gait_name, subgait=subgait_names, versions=versions))
        return result.success, result.message
        # except rospy.ServiceException:
        #     raise GaitServiceError

    def set_default_versions(self):
        """Save the current gait version map in the gait selection node as a default."""
        # try:
        while not self._get_version_map.wait_for_service(timeout_sec=2):
            self._node.get_logger().warn("Waiting for set_default_versions service to be available, is gait selection running?")
        result = self._set_default_versions.call(Trigger.Request())
        return result.success, result.message
        # except rospy.ServiceException:
        #     raise GaitServiceError

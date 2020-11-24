import ast
import os

import rclpy
import yaml
from std_srvs.srv import Trigger
from march_shared_msgs.srv import SetGaitVersion
from .gait_selection_errors import GaitServiceError, InvalidResponseError


class GaitSelectionController(object):
    def __init__(self, node, source_dir):
        """Base class to communicate with the gait selection node."""
        self._node = node
        self._source_dir = source_dir
        self._get_version_map = node.create_client(
            srv_type=Trigger, srv_name='/march/gait_selection/get_version_map')
        self._get_directory_structure = node.create_client(
            srv_type=Trigger, srv_name='/march/gait_selection/get_directory_structure')

        self._set_gait_version = node.create_client(
            srv_type=SetGaitVersion, srv_name='/march/gait_selection/set_gait_version')
        self._get_default_dict = node.create_client(
            srv_type=Trigger, srv_name='/march/gait_selection/get_default_dict')

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
            self._node.get_logger().info(f"Get directory service")
            result = dict(ast.literal_eval(self._get_directory_structure.call(Trigger.Request()).message))
            self._node.get_logger().info(f"{result}")
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
        while not self._set_gait_version.wait_for_service(timeout_sec=2):
            self._node.get_logger().warn("Waiting for set_gait_version service to be available, is gait selection running?")
        self._node.get_logger().info(f"Set_gait_version service: {gait_name}")
        self._node.get_logger().info(f"Set_gait_version service: {subgait_names}")
        self._node.get_logger().info(f"Set_gait_version service: {versions}")
        result = self._set_gait_version.call(
            SetGaitVersion.Request(gait=gait_name, subgaits=subgait_names, versions=versions))
        return result.success, result.message
        # except rospy.ServiceException:
        #     raise GaitServiceError

    def set_default_versions(self):
        """Save the current gait version map in the gait selection node as a default."""
        # try:
        while not self._get_default_dict.wait_for_service(timeout_sec=2):
            self._node.get_logger().warn("Waiting for set_default_versions service to be available, is gait selection running?")
        self._node.get_logger().info(f"Set default service 3 {self._source_dir}")

        # Temporary solution to find the march_gait_files source to change the
        # default versions
        file_path = os.path.join(os.path.dirname(self._source_dir), 'ros1', 'src', 'march_gait_files')
        self._node.get_logger().info(f"Set default service {file_path}")
        new_default_dict = dict(ast.literal_eval(self._get_directory_structure.call(Trigger.Request()).message))
        self._node.get_logger().info(f"Set default service {new_default_dict}")
        try:
            with open(file_path, 'w') as default_yaml_content:
                yaml_content = yaml.dump(new_default_dict, default_flow_style=False)
                default_yaml_content.write(yaml_content)

        except IOError:
            self._node.get_logger().warn('Error occurred when writing to file path: {pn}'.format(pn=self._default_yaml))

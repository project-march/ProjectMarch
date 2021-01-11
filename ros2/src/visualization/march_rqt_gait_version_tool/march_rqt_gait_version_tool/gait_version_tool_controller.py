import ast
import os
import yaml
from std_srvs.srv import Trigger
from march_shared_msgs.srv import SetGaitVersion
from .gait_version_tool_errors import InvalidResponseError

# Go directories up until you reach the ros2/ folder, then navigate to the
# march_gait_files package source
GAIT_SOURCE_DIR = os.path.join(
    os.path.dirname(__file__),
    "..",
    "..",
    "..",
    "..",
    "..",
    "..",
    "src",
    "gaits",
    "march_gait_files",
)


class GaitVersionToolController(object):
    def __init__(self, node):
        """Base class to communicate with the gait selection node."""
        self._node = node
        self._source_dir = GAIT_SOURCE_DIR
        self._get_version_map = node.create_client(
            srv_type=Trigger, srv_name="/march/gait_selection/get_version_map"
        )
        self._get_directory_structure = node.create_client(
            srv_type=Trigger, srv_name="/march/gait_selection/get_directory_structure"
        )

        self._set_gait_version = node.create_client(
            srv_type=SetGaitVersion, srv_name="/march/gait_selection/set_gait_version"
        )
        self._get_default_dict = node.create_client(
            srv_type=Trigger, srv_name="/march/gait_selection/get_default_dict"
        )
        self._gait_directory = self.get_current_gait_directory()

    def wait_for_service(self, service):
        while not service.wait_for_service(timeout_sec=2):
            self._node.get_logger().warn(
                f"Waiting for {service.srv_name} service to be available, "
                f"is gait selection running?"
            )

    def get_current_gait_directory(self):
        """Get the gait directory used by the gait selection node,
        to allow updating the default version."""
        gait_dir_client = self._node.create_client(
            srv_type=Trigger, srv_name="/march/gait_selection/get_gait_directory"
        )
        self.wait_for_service(gait_dir_client)
        gait_directory = gait_dir_client.call(Trigger.Request()).message
        gait_dir_client.destroy()
        return gait_directory

    def get_version_map(self):
        """Get the gait version map used in the gait selection node."""
        try:
            self.wait_for_service(self._get_version_map)
            return dict(
                ast.literal_eval(self._get_version_map.call(Trigger.Request()).message)
            )
        except ValueError:
            raise InvalidResponseError

    def get_directory_structure(self):
        """Get the gait directory of the selected gait_directory in the gait selection node."""
        try:
            self.wait_for_service(self._get_directory_structure)
            return dict(
                ast.literal_eval(
                    self._get_directory_structure.call(Trigger.Request()).message
                )
            )
        except ValueError:
            raise InvalidResponseError

    def set_gait_version(self, gait_name, subgait_names, versions):
        """Set a new gait version map to use in the gait selection node.

        :param str gait_name: The name of the gait
        :param list(str) subgait_names: Names of subgaits of which to change the version
        :param list(str) versions: Names of the versions
        """
        self.wait_for_service(self._set_gait_version)
        result = self._set_gait_version.call(
            SetGaitVersion.Request(
                gait=gait_name, subgaits=subgait_names, versions=versions
            )
        )
        return result.success, result.message

    def set_default_versions(self):
        """Save the current gait version map in the gait selection node as a default."""
        self.wait_for_service(self._get_default_dict)

        file_path = os.path.join(GAIT_SOURCE_DIR, self._gait_directory, "default.yaml")
        new_default_dict = dict(
            ast.literal_eval(self._get_default_dict.call(Trigger.Request()).message)
        )

        try:
            with open(file_path, "w") as default_yaml_content:
                yaml_content = yaml.dump(new_default_dict, default_flow_style=False)
                default_yaml_content.write(yaml_content)
            return True, f"Successfully updated default to file: {file_path}"

        except IOError:
            warning = f"Error occurred when writing to file path: {file_path}"
            self._node.get_logger().warn(warning)
            return False, warning

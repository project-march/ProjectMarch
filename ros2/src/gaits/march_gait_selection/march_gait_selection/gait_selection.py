import os
import rclpy
from march_gait_selection.dynamic_gaits.balance_gait import BalanceGait
from march_gait_selection.dynamic_gaits.semi_dynamic_setpoints_gait import (
    SemiDynamicSetpointsGait,
)
from march_shared_msgs.srv import SetGaitVersion, ContainsGait
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml
from march_utility.exceptions.gait_exceptions import GaitError, GaitNameNotFound
from march_utility.gait.subgait import Subgait
from std_msgs.msg import String
from std_srvs.srv import Trigger
from urdf_parser_py import urdf

from march_utility.utilities.node_utils import get_robot_urdf
from .state_machine.setpoints_gait import SetpointsGait

NODE_NAME = "gait_selection"


class GaitSelection(Node):
    """Base class for the gait selection module."""

    def __init__(self, gait_package=None, directory=None, robot=None):
        super().__init__(
            NODE_NAME, automatically_declare_parameters_from_overrides=True
        )
        self._balance_used = False
        try:
            if gait_package is None:
                gait_package = (
                    self.get_parameter("gait_package")
                    .get_parameter_value()
                    .string_value
                )
            if directory is None:
                directory = (
                    self.get_parameter("gait_directory")
                    .get_parameter_value()
                    .string_value
                )

            self._balance_used = (
                self.get_parameter("balance").get_parameter_value().bool_value
            )

        except ParameterNotDeclaredException:
            self.get_logger().error(
                "Gait selection node started without required parameters "
                "gait_package, gait_directory and balance"
            )

        package_path = get_package_share_directory(gait_package)
        self._directory_name = directory
        self._gait_directory = os.path.join(package_path, directory)
        self._default_yaml = os.path.join(self._gait_directory, "default.yaml")

        if not os.path.isdir(self._gait_directory):
            self.get_logger().error(f"Gait directory does not exist: {directory}")
        if not os.path.isfile(self._default_yaml):
            self.get_logger().error(
                f"Gait default yaml file does not exist: {directory}/default.yaml"
            )

        (
            self._gait_version_map,
            self._positions,
            self._semi_dynamic_gait_version_map,
        ) = self._load_configuration()
        self._robot = get_robot_urdf(self) if robot is None else robot

        self._robot_description_sub = self.create_subscription(
            msg_type=String,
            topic="/march/robot_description",
            callback=self._update_robot_description_cb,
            qos_profile=10,
        )

        self._create_services()
        self._loaded_gaits = self._load_gaits()
        self.get_logger().info("Successfully initialized gait selection node.")

    def _create_services(self) -> None:
        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_version_map",
            callback=lambda req, res: Trigger.Response(
                success=True, message=str(self.gait_version_map)
            ),
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_gait_directory",
            callback=lambda req, res: Trigger.Response(
                success=True, message=self._directory_name
            ),
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_default_dict",
            callback=self.get_default_dict_cb,
        )

        self.create_service(
            srv_type=SetGaitVersion,
            srv_name="/march/gait_selection/set_gait_version",
            callback=self.set_gait_versions_cb,
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_directory_structure",
            callback=lambda req, res: Trigger.Response(
                success=True, message=str(self.scan_directory())
            ),
        )

        self.create_service(
            srv_type=ContainsGait,
            srv_name="/march/gait_selection/contains_gait",
            callback=self.contains_gait_cb,
        )

    @property
    def robot(self):
        """ Return the robot obtained from the robot state publisher."""
        return self._robot

    @property
    def gait_version_map(self):
        """Returns the mapping from gaits and subgaits to versions."""
        return self._gait_version_map

    @property
    def positions(self):
        """Returns the named idle positions."""
        return self._positions

    def _update_robot_description_cb(self, msg):
        """
        Callback that is used to update the robot description when
        robot_state_publisher sends out an update.
        """
        self._robot = urdf.Robot.from_xml_string(msg.data)

    def set_gait_versions(self, gait_name, version_map):
        """Sets the subgait versions of given gait.

        :param str gait_name: Name of the gait to change versions
        :param dict version_map: Mapping subgait names to versions
        """
        self.get_logger().info(f"Setting gait versions, should be {version_map}")
        if gait_name not in self._loaded_gaits:
            raise GaitNameNotFound(gait_name)

        # Only update versions that are different
        version_map = dict(
            [
                (name, version)
                for name, version in version_map.items()
                if version != self._gait_version_map[gait_name][name]
            ]
        )
        self._loaded_gaits[gait_name].set_subgait_versions(
            self._robot, self._gait_directory, version_map
        )
        self._gait_version_map[gait_name].update(version_map)
        self.get_logger().info(
            f"Setting gait versions, is {self._gait_version_map[gait_name]}"
        )

    def set_gait_versions_cb(self, request, response):
        """Sets a new gait version to the gait selection instance.

        :type msg: march_shared_resources.srv.SetGaitVersionRequest

        :rtype march_shared_resources.srv.SetGaitVersionResponse
        """

        if len(request.subgaits) != len(request.versions):
            return [False, "`subgaits` and `versions` array are not of equal length"]

        version_map = dict(zip(request.subgaits, request.versions))
        try:
            self.get_logger().info(f"Setting gait versions from {request}")
            self.set_gait_versions(request.gait, version_map)
            response.success = True
            response.message = ""
            return response
        except Exception as e:  # noqa: PIE786
            response.success = False
            response.message = str(e)
            return response

    def contains_gait_cb(self, request, response):
        """
        Checks whether a gait and subgait are loaded.

        :type request: ContainsGaitRequest
        :param request: service request
        :return: True when the gait and subgait are loaded
        """
        gait = self._loaded_gaits.get(request.gait)
        if gait is None:
            response.contains = False
            return response

        response.contains = True
        for subgait in request.subgaits:
            if gait[subgait] is None:
                response.contains = False
        return response

    def scan_directory(self):
        """Scans the gait_directory recursively and create a dictionary of all
        subgait files.

        :returns:
            dictionary of the maps and files within the directory
        """
        gaits = {}
        for gait in os.listdir(self._gait_directory):
            gait_path = os.path.join(self._gait_directory, gait)

            if os.path.isdir(gait_path):
                subgaits = {}

                for subgait in os.listdir(gait_path):
                    subgait_path = os.path.join(gait_path, subgait)

                    if os.path.isdir(subgait_path):
                        versions = sorted(
                            [
                                v.replace(".subgait", "")
                                for v in os.listdir(os.path.join(subgait_path))
                                if v.endswith(".subgait")
                            ]
                        )
                        subgaits[subgait] = versions

                gaits[gait] = subgaits
        return gaits

    def get_default_dict_cb(self, req, res):
        defaults = {"gaits": self._gait_version_map, "positions": self._positions}
        return Trigger.Response(success=True, message=str(defaults))

    def add_gait(self, gait):
        """Adds a gait to the loaded gaits if it does not already exist.

        The to be added gait should implement `GaitInterface`.
        """
        if gait.name in self._loaded_gaits:
            self.get_logger().warn(
                "Gait `{gait}` already exists in gait selection".format(gait=gait.name)
            )
        else:
            self._loaded_gaits[gait.name] = gait

    def _load_gaits(self):
        """Loads the gaits in the specified gait directory.

        :returns dict: A dictionary mapping gait name to gait instance
        """
        gaits = {}

        for gait in self._gait_version_map:
            gaits[gait] = SetpointsGait.from_file(
                gait, self._gait_directory, self._robot, self._gait_version_map
            )

        self._load_semi_dynamic_gaits(gaits)

        if self._balance_used and "balance_walk" in gaits.keys():
            balance_gait = BalanceGait(node=self, default_walk=gaits["balance_walk"])
            if balance_gait is not None:
                self.get_logger().info("Successfully created a balance gait")
                gaits["balanced_walk"] = balance_gait

        return gaits

    def _load_semi_dynamic_gaits(self, gaits):
        """
        Loads the semi dynamic gaits, this is currently only 1 gait.
        :param gaits: dict to add the semi dynamic gaits to
        """
        for gait in self._semi_dynamic_gait_version_map:
            gaits[f"dynamic_{gait}"] = SemiDynamicSetpointsGait.from_file(
                gait,
                self._gait_directory,
                self._robot,
                self._semi_dynamic_gait_version_map,
            )

    def _load_configuration(self):
        """Loads and verifies the gaits configuration."""
        with open(self._default_yaml, "r") as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)

        version_map = default_config["gaits"]
        semi_dynamic_version_map = default_config.get("semi_dynamic_gaits", [])

        if not isinstance(version_map, dict):
            raise TypeError("Gait version map should be of type; dictionary")

        if not self._validate_version_map(version_map):
            raise GaitError(
                msg="Gait version map: {gm}, is not valid".format(gm=version_map)
            )

        return version_map, default_config["positions"], semi_dynamic_version_map

    def _validate_version_map(self, version_map):
        """Validates if the current versions exist.

        :param dict version_map: Version map to verify
        :returns bool: True when all versions exist, False otherwise
        """
        for gait_name in version_map:
            gait_path = os.path.join(self._gait_directory, gait_name)
            if not os.path.isfile(os.path.join(gait_path, gait_name + ".gait")):
                self.get_logger().warn("gait {gn} does not exist".format(gn=gait_name))
                return False

            for subgait_name in version_map[gait_name]:
                version = version_map[gait_name][subgait_name]
                if not Subgait.validate_version(gait_path, subgait_name, version):
                    self.get_logger().warn(
                        "{0}, {1} does not exist".format(subgait_name, version)
                    )
                    return False
        return True

    def __getitem__(self, name):
        """Returns a gait from the loaded gaits."""
        return self._loaded_gaits.get(name)

    def __iter__(self):
        """Returns an iterator over all loaded gaits."""
        return iter(self._loaded_gaits.values())

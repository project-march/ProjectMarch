"""Author: ???."""

import os
from typing import Optional, List, Tuple, Dict, Union

import yaml
from ament_index_python.packages import get_package_share_directory
from march_gait_selection.gaits.balance_gait import BalanceGait
from march_gait_selection.gaits.dynamic_edge_setpoints_gait import (
    DynamicEdgeSetpointsGait,
)
from march_shared_msgs.srv import SetGaitVersion, ContainsGait, GetGaitParameters

from march_utility.exceptions.gait_exceptions import (
    GaitError,
    GaitNameNotFoundError,
    NonValidGaitContentError,
)
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import (
    get_robot_urdf_from_service,
    get_joint_names_from_robot,
    DEFAULT_HISTORY_DEPTH,
)
from march_utility.utilities.utility_functions import (
    validate_and_get_joint_names_for_inverse_kinematics,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.exceptions import ParameterNotDeclaredException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from urdf_parser_py import urdf

from march_gait_selection.gaits.realsense_gait import RealsenseGait
from march_gait_selection.gaits.setpoints_gait import SetpointsGait
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,
)
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step import (
    DynamicSetpointGaitStep,
)
from march_gait_selection.dynamic_interpolation.cybathlon_obstacle_gaits.stepping_stones_step_and_close import (
    SteppingStonesStepAndClose,
)
from march_gait_selection.dynamic_interpolation.cybathlon_obstacle_gaits.dynamic_setpoint_gait_step_and_hold import (
    DynamicSetpointGaitStepAndHold,
)
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_close import (
    DynamicSetpointGaitClose,
)

NODE_NAME = "gait_selection"


class GaitSelection(Node):
    """Base class for the gait selection module.

    Args:
        gait_package (Optional[str]): path to the package that contains the gait files, defaults to launch argument
        directory (Optional[str]): name of the directory that contains the gait files, defaults to launch argument
        robot (Optional[urdf.Robot]): robot to use, defaults to launch argument
        balance (Optional[bool]): whether to create the balance gait, defaults to launch argument
        dynamic_gait (Optional[bool]): whether to create the dynamic setpoint gaits, defaults to launch argument
    Attributes:
        logger (Logger): used to log to the terminal
        middle_point_fraction (float): fraction of the step at which the middle setpoint will be set
        middle_point_height (float): height in meters that the middle point will be above the end point
        minimum_stair_height (float): steps higher or lower will have the gait_type 'stairs_like' instead of 'walk-like'
        push_off_fraction (float): fraction of the step at which the push off setpoint will be set
        push_off_position (float): position (in rad) of the ankle joint that will be set during push off

        _balance_used (bool): whether balance gait will be created
        _dynamic_gait (bool): whether dynamic setpoint gaits will be created
        _early_schedule_duration (Duration): duration to use for early scheduling
        _first_subgait_delay (Duration): delay with which first subgait will be early scheduled
        _directory_name (str): name of the directory that contains the gait files
        _gait_package (str): path to the package that contains the gait files
        _gait_directory (str): ??? TODO: Add docs
        _default_yaml (str): ??? TODO: Add  docs
        _robot (urdf.Robot): robot model to use
        _joint_names (List[str]): alphabetical list of joint names of _robot
        _realsense_yaml (str): path to the yaml file containing realsense parameters
        _realsense_gait_version_map (str): ??? TODO: Add docs
        _gait_version_map (Dict): ??? TODO: Add docs
        _positions (Dict): ??? TODO: Add docs
        _dynamic_edge_version_map (Dict[Any, Dict[str, dict]], dict]): ??? TODO: Add docs
        _robot_description_sub (Subscriber): subscribes to /march/robot_description
        _gaits (Dict): dictionary containing all possible gaits
        _loaded_gaits (dict): dictionary containing all possible gaits TODO: difference between _gaits and _loaded_gaits

    Raises:
        ParameterNotDeclaredException: raised when gait_selection_node is not initialized with all required params
        FileNotFoundError: raised when gait_directory path does not exist
    """

    _loaded_gaits: dict

    def __init__(
        self,
        gait_package: Optional[str] = None,
        directory: Optional[str] = None,
        robot: Optional[urdf.Robot] = None,
        balance: Optional[bool] = None,
        dynamic_gait: Optional[bool] = None,
    ):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)
        self.logger = Logger(self, __class__.__name__)
        self._balance_used = False
        self._dynamic_gait = False
        try:
            # Initialize all parameters once, and set up a callback for dynamically
            # reconfiguring
            if gait_package is None:
                gait_package = self.get_parameter("gait_package").get_parameter_value().string_value
            if directory is None:
                directory = self.get_parameter("gait_directory").get_parameter_value().string_value
            if balance is None:
                self._balance_used = self.get_parameter("balance").get_parameter_value().bool_value
            if dynamic_gait is None:
                self._dynamic_gait = self.get_parameter("dynamic_gait").get_parameter_value().bool_value

            self._early_schedule_duration = self._parse_duration_parameter("early_schedule_duration")
            self._first_subgait_delay = self._parse_duration_parameter("first_subgait_delay")

            # Setting dynamic gait parameters
            self.middle_point_fraction = self.get_parameter("middle_point_fraction").get_parameter_value().double_value
            self.middle_point_height = self.get_parameter("middle_point_height").get_parameter_value().double_value
            self.minimum_stair_height = self.get_parameter("minimum_stair_height").get_parameter_value().double_value
            self.push_off_fraction = self.get_parameter("push_off_fraction").get_parameter_value().double_value
            self.push_off_position = self.get_parameter("push_off_position").get_parameter_value().double_value
            self.add_push_off = self.get_parameter("add_push_off").get_parameter_value().bool_value
            self.use_position_queue = self.get_parameter("use_position_queue").get_parameter_value().bool_value
            self.amount_of_steps = self.get_parameter("amount_of_steps").get_parameter_value().integer_value
            self._add_cybathlon_gaits = self.get_parameter("add_cybathlon_gaits").get_parameter_value().bool_value

        except ParameterNotDeclaredException:
            self.logger.error(
                "Gait selection node started without required parameters gait_package, gait_directory and balance"
            )

        self._directory_name = directory
        self._gait_package = gait_package
        self._gait_directory, self._default_yaml = self._initialize_gaits()
        if not os.path.isdir(self._gait_directory):
            self.logger.error(f"Gait directory does not exist: {directory}")
            raise FileNotFoundError(directory)
        if not os.path.isfile(self._default_yaml):
            self.logger.error(f"Gait default yaml file does not exist: {directory}/default.yaml")

        self._robot = get_robot_urdf_from_service(self) if robot is None else robot
        self._joint_names = sorted(get_joint_names_from_robot(self._robot))

        self._realsense_yaml = os.path.join(self._gait_directory, "realsense_gaits.yaml")

        self._realsense_gait_version_map = self._load_realsense_configuration()
        (
            self._gait_version_map,
            self._positions,
            self._dynamic_edge_version_map,
        ) = self._load_configuration()

        self._robot_description_sub = self.create_subscription(
            msg_type=String,
            topic="/robot_description",
            callback=self._update_robot_description_cb,
            qos_profile=DEFAULT_HISTORY_DEPTH,
        )

        self._create_services()
        self._gaits = {}
        self._gaits = self._load_gaits()

        self._early_schedule_duration = self._parse_duration_parameter("early_schedule_duration")
        self._first_subgait_delay = self._parse_duration_parameter("first_subgait_delay")

        if not self._validate_inverse_kinematics_is_possible():
            self.logger.warning(
                "The currently available joints are unsuitable for "
                "using inverse kinematics.\n"
                "Any interpolation on foot_location will return "
                "the base subgait instead. Realsense gaits will "
                "not be loaded."
            )
        self.logger.info("Successfully initialized gait selection node.")

    @property
    def joint_names(self) -> List[str]:
        """Return a list containing joint names."""
        return self._joint_names

    @property
    def gaits(self) -> dict:
        """Return a dictionary containing the loaded gaits."""
        return self._gaits

    def _validate_inverse_kinematics_is_possible(self) -> bool:
        """Whether inverse kinematics is possible."""
        return validate_and_get_joint_names_for_inverse_kinematics(self.logger) is not None

    def _initialize_gaits(self) -> Tuple[str, str]:
        """Initialize the gait packages."""
        package_path = get_package_share_directory(self._gait_package)
        gait_directory = os.path.join(package_path, self._directory_name)
        default_yaml = os.path.join(gait_directory, "default.yaml")

        if not os.path.isdir(gait_directory):
            self.logger.error(f"Gait directory does not exist: {gait_directory}")
        if not os.path.isfile(default_yaml):
            self.logger.error(f"Gait default yaml file does not exist: {gait_directory}/default.yaml")
        return gait_directory, default_yaml

    def update_gaits(self) -> None:
        """Update the gaits after one of the gait attributes has been changed."""
        self._gait_directory, self._default_yaml = self._initialize_gaits()
        self._realsense_yaml = os.path.join(self._gait_directory, "realsense_gaits.yaml")

        self._realsense_gait_version_map = self._load_realsense_configuration()
        (
            self._gait_version_map,
            self._positions,
            self._semi_dynamic_gait_version_map,
        ) = self._load_configuration()

        self._loaded_gaits = self._load_gaits()

    def _create_services(self) -> None:
        """Create services for gait_selection."""
        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_version_map",
            callback=lambda req, res: Trigger.Response(success=True, message=str(self.gait_version_map)),
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="/march/gait_selection/get_gait_directory",
            callback=lambda req, res: Trigger.Response(success=True, message=self._directory_name),
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
            callback=lambda req, res: Trigger.Response(success=True, message=str(self.scan_directory())),
        )

        self.create_service(
            srv_type=ContainsGait,
            srv_name="/march/gait_selection/contains_gait",
            callback=self.contains_gait_cb,
        )

    def _parse_duration_parameter(self, name: str) -> Duration:
        """Get a duration parameter from the parameter server.

        Returns:
            Duration: duration of the parameter given by name. If param does not exist or is negative, returns zero
        """
        if self.has_parameter(name):
            value = self.get_parameter(name).value
            if value < 0:
                value = 0
            return Duration(seconds=value)
        else:
            return Duration(0)

    def shortest_subgait(self) -> Subgait:
        """Get the subgait with the smallest duration of all subgaits in the loaded gaits.

        Returns:
            Subgait: subgait with the shortest duration
        """
        shortest_subgait = None
        for gait in self._gaits.values():
            for subgait in gait.subgaits.values():
                if shortest_subgait is None or subgait.duration < shortest_subgait.duration:
                    shortest_subgait = subgait
        return shortest_subgait

    @property
    def robot(self) -> urdf.Robot:
        """Return the robot obtained from the robot state publisher."""
        return self._robot

    @property
    def gait_version_map(self) -> dict:
        """Returns the mapping from gaits and subgaits to versions."""
        return self._gait_version_map

    @property
    def positions(self) -> dict:
        """Returns the named idle positions."""
        return self._positions

    def _update_robot_description_cb(self, msg: String) -> None:
        """Callback that is used to update the robot description when robot_state_publisher sends out an update."""
        self._robot = urdf.Robot.from_xml_string(msg.data)

    def set_gait_versions(self, gait_name: str, version_map: Dict[str, str]):
        """Sets the subgait versions of given gait.

        Args:
            gait_name (str): Name of the gait to change versions
            version_map (Dict[str, str]): Mapping subgait names to versions
        """
        if gait_name not in self._gaits:
            raise GaitNameNotFoundError(gait_name)

        # Only update versions that are different
        version_map = {
            name: version for name, version in version_map.items() if version != self._gait_version_map[gait_name][name]
        }
        self._gaits[gait_name].set_subgait_versions(self._robot, self._gait_directory, version_map)
        self._gait_version_map[gait_name].update(version_map)
        self.logger.info(f"Setting gait versions successful: {self._gaits[gait_name]}")

    def set_gait_versions_cb(self, request, response) -> List[Union[bool, str]]:
        """Sets a new gait version to the gait selection instance.

        Args:
            request (SetGaitVersionRequest): service request
            response (SetGaitVersionResponse): response to service request
        Returns:
            List[Union[bool, str]]: march_shared_resources.srv.SetGaitVersionResponse
        Raises:
            Exception: raised when gait version cannot be set
        """
        if len(request.subgaits) != len(request.versions):
            return [False, "`subgaits` and `versions` array are not of equal length"]

        version_map = dict(zip(request.subgaits, request.versions))
        try:
            self.logger.info(f"Setting gait versions from {request}")
            self.set_gait_versions(request.gait, version_map)
            response.success = True
            response.message = ""
            return response
        except Exception as e:  # noqa: PIE786 TODO: create specific exception for this
            response.success = False
            response.message = str(e)
            return response

    def contains_gait_cb(self, request, response) -> bool:
        """Checks whether a gait and subgait are loaded.

        Args:
            request (ContainsGaitRequest): service request
            response (ContainsGaitResponse): response to service request
        Returns:
             bool: True when the gait and subgait are loaded
        """
        gait = self._gaits.get(request.gait)
        if gait is None:
            response.contains = False
            return response

        response.contains = True
        for subgait in request.subgaits:
            if gait[subgait] is None:
                response.contains = False
        return response

    def scan_directory(self) -> Dict[str, Dict[str, List[str]]]:  # noqa TAE002 suppress to complex expression
        """Scans the gait_directory recursively and create a dictionary of all subgait files.

        Returns:
            Dict[str, Dict[str, List[str]]]: dictionary of the maps and files within the directory
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

    def get_default_dict_cb(self, request, response):
        """Service that returns the default gaits and positions.

        Args:
            request (TriggerRequest): service request
            response (TriggerResponse): response to service request
        Returns:
             Trigger.Response
        """
        defaults = {"gaits": self._gait_version_map, "positions": self._positions}
        return Trigger.Response(success=True, message=str(defaults))

    def add_gait(self, gait):
        """Adds a gait to the loaded gaits if it does not already exist.

        Args:
            gait: gait class that should be added to the loaded gaits. The to be added gait should implement
                `GaitInterface`
        """
        if gait.name in self._gaits:
            self.logger.warning("Gait `{gait}` already exists in gait selection".format(gait=gait.name))
        else:
            self._gaits[gait.name] = gait

    def _load_gaits(self) -> dict:
        """Loads the gaits in the specified gait directory.

        Returns:
            dict: A dictionary mapping gait name to gait instance
        """
        gaits = {}

        for gait in self._gait_version_map:
            gaits[gait] = SetpointsGait.from_file(gait, self._gait_directory, self._robot, self._gait_version_map)

        for gait in self._dynamic_edge_version_map:
            self.logger.debug(f"Adding dynamic gait {gait}")
            start_is_dynamic = self._dynamic_edge_version_map[gait].pop("start_is_dynamic", True)
            final_is_dynamic = self._dynamic_edge_version_map[gait].pop("final_is_dynamic", True)
            gaits[gait] = DynamicEdgeSetpointsGait.dynamic_from_file(
                gait,
                self._gait_directory,
                self._robot,
                self._dynamic_edge_version_map,
                start_is_dynamic,
                final_is_dynamic,
            )
            self._gait_version_map[gait] = self._dynamic_edge_version_map[gait]
        self._load_realsense_gaits(gaits)
        if self._balance_used and "balance_walk" in gaits:
            balance_gait = BalanceGait(node=self, default_walk=gaits["balance_walk"])
            if balance_gait is not None:
                self.logger.info("Successfully created a balance gait")
                gaits["balanced_walk"] = balance_gait

        if self._dynamic_gait:
            # We pass along the gait_selection_node to be able to listen to the CoViD topic within the
            # DynamicSetpointGait class. Dynamic setpoint gait needs to be an attribute for updating parameters.
            self.dynamic_setpoint_gait = DynamicSetpointGait(gait_selection_node=self)
            self.dynamic_setpoint_gait_step = DynamicSetpointGaitStep(gait_selection_node=self)
            self.dynamic_setpoint_gait_step_and_hold = DynamicSetpointGaitStepAndHold(gait_selection_node=self)

            dynamic_gaits = [
                self.dynamic_setpoint_gait,
                self.dynamic_setpoint_gait_step,
                DynamicSetpointGaitStepAndClose(gait_selection_node=self),
                DynamicSetpointGaitClose(gait_selection_node=self),
            ]

            if self._add_cybathlon_gaits:
                dynamic_gaits.append(self.dynamic_setpoint_gait_step_and_hold)
                dynamic_gaits.append(SteppingStonesStepAndClose(gait_selection_node=self))

            for dynamic_gait in dynamic_gaits:
                gaits[dynamic_gait.name] = dynamic_gait

            self.logger.info("Added dynamic_walk to gaits")

        return gaits

    def _load_realsense_gaits(self, gaits) -> None:
        """Load all gaits from the realsense gait version map.

        Also create a service with a separate callback group that can be used by the
        realsense gaits to get parameters from the realsense_reader. A new callback
        group is necessary to prevent a deadlock.

        Args:
            gaits (dict): The dictionary where the loaded gaits will be added to.
        """
        if not self._validate_inverse_kinematics_is_possible():
            return
        get_gait_parameters_service = self.create_client(
            srv_type=GetGaitParameters,
            srv_name="/camera/process_pointcloud",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        for gait_name in self._realsense_gait_version_map:
            gait_folder = gait_name
            gait_path = os.path.join(self._gait_directory, gait_folder, gait_name + ".gait")
            with open(gait_path, "r") as gait_file:
                gait_graph = yaml.load(gait_file, Loader=yaml.SafeLoader)["subgaits"]
            gait = RealsenseGait.from_yaml(
                gait_selection=self,
                robot=self._robot,
                gait_name=gait_name,
                gait_config=self._realsense_gait_version_map[gait_name],
                gait_graph=gait_graph,
                gait_directory=self._gait_directory,
                process_service=get_gait_parameters_service,
            )
            gaits[gait_name] = gait

    def _load_realsense_configuration(self) -> dict:
        """Load the realsense configuration from the yaml file.

        Returns:
            dict: dictionary containing the information in the yaml file
        """
        if not os.path.isfile(self._realsense_yaml):
            self.logger.info("No realsense_yaml present, no realsense gaits will be created.")
            return {}
        with open(self._realsense_yaml, "r") as realsense_config_file:
            return yaml.load(realsense_config_file, Loader=yaml.SafeLoader)

    def _load_configuration(self) -> Tuple[dict, dict, dict]:
        """Loads and verifies the gaits configuration.

        Raises:
            TypeError: raised when gait version map is not a dictionary
            GaitError: raised when gait version map is not valid
            NonValidGaitContentError: raised when the position dictionary does not contain positions for each joint
        """
        with open(self._default_yaml, "r") as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)

        version_map = default_config["gaits"]
        dynamic_edge_version_map = {}
        if "dynamic_edge_gaits" in default_config:
            dynamic_edge_version_map = default_config["dynamic_edge_gaits"]

        if not isinstance(version_map, dict):
            raise TypeError("Gait version map should be of type; dictionary")

        if not self._validate_version_map(version_map):
            raise GaitError(msg="Gait version map: {gm}, is not valid".format(gm=version_map))

        positions = {}

        for position_name, position_values in default_config["positions"].items():
            positions[position_name] = {
                "gait_type": position_values["gait_type"],
                "joints": {},
            }
            for joint, joint_value in position_values["joints"].items():
                if joint in self._joint_names:
                    positions[position_name]["joints"][joint] = joint_value

            if set(positions[position_name]["joints"].keys()) != set(self._joint_names):
                raise NonValidGaitContentError(
                    f"The position {position_name} does not "
                    f"have a position for all required joints: it "
                    f"has {positions[position_name]['joints'].keys()}, "
                    f"required: {self._joint_names}"
                )
        return version_map, positions, dynamic_edge_version_map

    def _validate_version_map(self, version_map) -> bool:
        """Validates if the current versions exist.

        Args:
            version_map (dict): version map to verify
        Returns:
            bool: True if version map is valid, else False
        """
        for gait_name in version_map:
            gait_path = os.path.join(self._gait_directory, gait_name)
            if not os.path.isfile(os.path.join(gait_path, gait_name + ".gait")):
                self.logger.warning("gait {gn} does not exist".format(gn=gait_name))
                return False

            for subgait_name in version_map[gait_name]:
                version = version_map[gait_name][subgait_name]
                if not Subgait.validate_version(gait_path, subgait_name, version):
                    self.logger.warning("{0}, {1} does not exist".format(subgait_name, version))
                    return False
        return True

    def get_named_position(self, position: str) -> Dict[str, float]:
        """Returns a joint dict of a named position from the gait_selection node.

        Args:
            position (str): name of the position
        Returns:
            Dict[str, float]: a dict containing joint names and positions for the actuating joints.
        """
        return self.positions[position]["joints"]

    def __getitem__(self, name: str):
        """Returns a gait from the loaded gaits.

        Returns:
            The gait class corresponding to the name
        """
        return self._gaits.get(name)

    def __iter__(self):
        """Returns an iterator over all loaded gaits."""
        return iter(self._gaits.values())

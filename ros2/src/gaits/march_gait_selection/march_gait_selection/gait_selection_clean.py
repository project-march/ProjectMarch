"""Author: Marten Haitjema."""

import os
from typing import List, Optional, Dict

import yaml
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from urdf_parser_py import urdf

from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_hold import (
    DynamicSetpointGaitStepAndHold,
)
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import DynamicSetpointGait
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_close import DynamicSetpointGaitClose
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step import DynamicSetpointGaitStep
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait_step_and_close import (
    DynamicSetpointGaitStepAndClose,
)
from march_gait_selection.gaits.home_gait import HomeGait
from march_gait_selection.gaits.setpoints_gait import SetpointsGait
from march_utility.exceptions.gait_exceptions import NonValidGaitContentError
from march_utility.gait.edge_position import UnknownEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import get_joint_names_from_robot, get_robot_urdf_from_service

NODE_NAME = "gait_selection"
UNKNOWN = "unknown"


class GaitSelectionClean(Node):
    """Base class for the gait selection module."""

    def __init__(
        self,
        gait_package: Optional[str] = None,
        directory: Optional[str] = None,
        robot: Optional[urdf.Robot] = None,
        dynamic_gait: Optional[bool] = None,
    ):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)
        self.logger = Logger(self, __class__.__name__)
        self._robot = self._robot = get_robot_urdf_from_service(self) if robot is None else robot
        self._joint_names = sorted(get_joint_names_from_robot(self._robot))
        self._get_gait_parameters(gait_package, directory, dynamic_gait)

        package_path = get_package_share_directory(self._gait_package)
        self._gait_directory = os.path.join(package_path, self._directory_name)
        self._default_positions_yaml = os.path.join(self._gait_directory, "default_positions.yaml")
        self._loaded_gaits = {}
        self._named_positions = {}
        self._load_gaits()

    @property
    def joint_names(self) -> List[str]:
        """Return a list containing joint names."""
        return self._joint_names

    @property
    def gaits(self) -> dict:
        """Return a dictionary containing the loaded gaits."""
        return self._loaded_gaits

    @property
    def robot(self) -> urdf.Robot:
        """Return the robot obtained from the robot state publisher."""
        return self._robot

    @property
    def positions(self) -> dict:
        """Returns the named idle positions."""
        return self._named_positions

    def _get_gait_parameters(self, gait_package: str, directory: str, dynamic_gait: bool) -> None:
        try:
            # Initialize all parameters once, and set up a callback for dynamically
            # reconfiguring
            if gait_package is None:
                self._gait_package = self.get_parameter("gait_package").get_parameter_value().string_value
            if directory is None:
                self._directory_name = self.get_parameter("gait_directory").get_parameter_value().string_value
            if dynamic_gait is None:
                self._dynamic_gait = self.get_parameter("dynamic_gait").get_parameter_value().bool_value

            self.middle_point_fraction = self.get_parameter("middle_point_fraction").get_parameter_value().double_value
            self.middle_point_height = self.get_parameter("middle_point_height").get_parameter_value().double_value
            self.minimum_stair_height = self.get_parameter("minimum_stair_height").get_parameter_value().double_value
            self.push_off_fraction = self.get_parameter("push_off_fraction").get_parameter_value().double_value
            self.push_off_position = self.get_parameter("push_off_position").get_parameter_value().double_value
            self.add_push_off = self.get_parameter("add_push_off").get_parameter_value().bool_value
            self.use_position_queue = self.get_parameter("use_position_queue").get_parameter_value().bool_value
            self.amount_of_steps = self.get_parameter("amount_of_steps").get_parameter_value().integer_value
            self._early_schedule_duration = self._parse_duration_parameter("early_schedule_duration")
            self._first_subgait_delay = self._parse_duration_parameter("first_subgait_delay")
        except ParameterNotDeclaredException:
            self.logger.error(
                "Gait selection node started without required parameters gait_package, gait_directory and balance"
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

    def _load_gaits(self) -> None:
        """Load all gaits."""
        self._load_named_positions()
        self._load_dynamic_gaits()
        self._load_home_gaits()
        self._load_sit_and_stand_gaits()

    def _load_dynamic_gaits(self) -> None:
        """Load the dynamic gait classes."""
        # Some gaits need to be an attribute to be able to reconfigure parameters during run time.
        self.dynamic_setpoint_gait = DynamicSetpointGait(gait_selection_node=self)
        self.dynamic_setpoint_gait_step = DynamicSetpointGaitStep(gait_selection_node=self)
        self.dynamic_setpoint_gait_step_and_hold = DynamicSetpointGaitStepAndHold(gait_selection_node=self)

        dynamic_gaits = [
            self.dynamic_setpoint_gait,
            self.dynamic_setpoint_gait_step,
            self.dynamic_setpoint_gait_step_and_hold,
            DynamicSetpointGaitStepAndClose(gait_selection_node=self),
            DynamicSetpointGaitClose(gait_selection_node=self),
        ]

        for dynamic_gait in dynamic_gaits:
            self._loaded_gaits[dynamic_gait.name] = dynamic_gait

    def _load_named_positions(self) -> None:
        """Load the named positions from default.yaml."""
        with open(self._default_positions_yaml, "r") as default_positions_yaml_file:
            default_config = yaml.load(default_positions_yaml_file, Loader=yaml.SafeLoader)

        self._gait_version_map = default_config["gaits"]

        for position_name, position_values in default_config["positions"].items():
            self._named_positions[position_name] = {
                "gait_type": position_values["gait_type"],
                "joints": {},
            }
            for joint, joint_value in position_values["joints"].items():
                if joint in self._joint_names:
                    self._named_positions[position_name]["joints"][joint] = joint_value

            if set(self._named_positions[position_name]["joints"].keys()) != set(self._joint_names):
                raise NonValidGaitContentError(
                    f"The position {position_name} does not have a position for all required joints: it "
                    f"has {self._named_positions[position_name]['joints'].keys()}, required: {self._joint_names}"
                )

    def _load_home_gaits(self) -> None:
        """Create the home gaits based on the named positions."""
        for name in self._named_positions.keys():
            position = self._named_positions[name]["joints"]
            if isinstance(position, UnknownEdgePosition):
                continue
            home_gait = HomeGait(name, position, "")
            self.gaits[home_gait.name] = home_gait

    def _load_sit_and_stand_gaits(self) -> None:
        """Loads the sit and stand gaits."""
        for gait in self._gait_version_map:
            self._loaded_gaits[gait] = SetpointsGait.from_file(
                gait, self._gait_directory, self._robot, self._gait_version_map
            )

    def get_named_position(self, position: str) -> Dict[str, float]:
        """Returns a joint dict of a named position from the gait_selection node.

        Args:
            position (str): name of the position
        Returns:
            Dict[str, float]: a dict containing joint names and positions for the actuating joints.
        """
        return self.positions[position]["joints"]

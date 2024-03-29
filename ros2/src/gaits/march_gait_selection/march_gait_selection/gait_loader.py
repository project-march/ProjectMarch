"""Author: Marten Haitjema."""

import os
import yaml

from typing import List, Dict
from ament_index_python import get_package_share_directory
from rclpy.node import Node

from march_gait_selection.dynamic_interpolation.point_handlers.simulated_point_handler import (
    SimulatedPointHandler,
)
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_walk import DynamicGaitWalk
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_step_and_close import DynamicGaitStepAndClose
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_step import DynamicGaitStep

from march_gait_selection.gaits.home_gait import HomeGait
from march_gait_selection.gaits.setpoints_gait import SetpointsGait
from march_utility.gait.edge_position import UnknownEdgePosition, StaticEdgePosition, EdgePosition
from march_utility.utilities.utility_functions import get_joint_names_from_urdf

NODE_NAME = "gait_selection"
UNKNOWN = "unknown"


class GaitLoader:
    """Class that returns a dictionary of all loaded gaits and positions.

    Args:
        node (rclpy.Node): the gait node

    Attributes:
        _node (rclpy.Node): the gait node
        _logger (rclpy.Logger): used to log to the terminal
        _actuating_joint_names (List[str]): a list of names of the actuating joints in the urdf
        _gait_directory (str): path to the directory that contains the gait files
        _default_yaml (str): path to the yaml that contains the named positions
        _loaded_gaits (Dict[str, Gait]): dictionary containing the name and instance of each loaded gait class
        _named_positions (Dict[EdgePosition, str]): dictionary containing the EdgePosition and name of each named pos
    """

    def __init__(
        self,
        node: Node,
    ):
        self._node = node
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._node.get_logger().info("Starting initializing gait loader")

        self._actuating_joint_names = get_joint_names_from_urdf()

        self._node.get_logger().info("Actuating gaits are:" + str(self._actuating_joint_names))

        package_path = get_package_share_directory(self._node.gait_package)
        self._gait_directory = os.path.join(package_path, self._node.directory_name)
        self._default_yaml = os.path.join(self._gait_directory, "default.yaml")
        self._loaded_gaits = {}
        self._named_positions = {}
        self._load_gaits()

    @property
    def joint_names(self) -> List[str]:
        """Return a list containing joint names."""
        return self._actuating_joint_names

    @property
    def gaits(self) -> dict:
        """Return a dictionary containing the loaded gaits."""
        return self._loaded_gaits

    @property
    def positions(self) -> Dict[EdgePosition, str]:
        """Returns the named idle positions."""
        return self._named_positions

    def _load_gaits(self) -> None:
        """Load all gaits."""
        self._load_named_positions()
        self._load_dynamic_gaits()
        self._load_home_gaits()
        self._load_sit_and_stand_gaits()

    def _load_dynamic_gaits(self) -> None:
        """Load the dynamic gait classes."""
        simulated_point_handler = SimulatedPointHandler(self._node)
        dynamic_gaits = [
            DynamicGaitWalk("fixed_walk", self._node, simulated_point_handler),
            DynamicGaitStepAndClose("fixed_step_and_close", self._node, simulated_point_handler),
            DynamicGaitStep("fixed_step", self._node, simulated_point_handler),
        ]

        self._loaded_gaits = {gait.name: gait for gait in dynamic_gaits}

    def _load_named_positions(self) -> None:
        """Load the named positions from default.yaml."""
        with open(self._default_yaml, "r") as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)

        self._gait_version_map = default_config["gaits"]

        for position_name, position_values in default_config["positions"].items():
            edge_position = StaticEdgePosition(
                [position_values["joints"][joint_name] for joint_name in self._actuating_joint_names]
            )
            if edge_position not in self._named_positions:
                self._named_positions[edge_position] = position_name
            else:
                self._logger.error(
                    f"Position '{position_name}' with joint values {position_values} cannot be added because it will "
                    f"overwrite position '{self._named_positions[edge_position]}'. "
                    f"Each named position should be unique."
                )

    def _load_home_gaits(self) -> None:
        """Create the home gaits based on the named positions."""
        for position in self._named_positions:
            if isinstance(position, UnknownEdgePosition):
                continue
            name = self._named_positions[position]
            home_gait = HomeGait(name, position, "")
            self.gaits[home_gait.name] = home_gait

    def _load_sit_and_stand_gaits(self) -> None:
        """Loads the sit and stand gaits."""
        for gait in self._gait_version_map:
            self._loaded_gaits[gait] = SetpointsGait.from_file(
                gait, self._gait_directory, self._gait_version_map
            )

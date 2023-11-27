"""Author: Marco Bak MVIII."""

import os
import yaml

from typing import List, Dict
from ament_index_python import get_package_share_directory
from rclpy.node import Node

from gait_selection.setpoints_gait import SetpointsGait
from gait_selection.home_gait import HomeGait
from march_utility.gait.edge_position import UnknownEdgePosition, StaticEdgePosition, EdgePosition


class GaitLoader:
    """Class that returns a dictionary of all loaded gaits and positions.

    Args:
        node (rclpy.Node): the gait node
    Attributes:
        _node (rclpy.Node): the gait node
        _logger (rclpy.Logger): used to log to the terminal
        _gait_directory (str): path to the directory that contains the gait files
        _default_yaml (str): path to the yaml that contains the named positions
        _loaded_gaits (Dict[str, Gait]): dictionary containing the name and instance of each loaded gait class
        _named_positions (Dict[EdgePosition, str]): dictionary containing the EdgePosition and name of each named pos
    """

    def __init__(
        self,
        node: Node,
    ):
        """Init the gait loader for the gait_selection."""
        self._node = node
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._actuating_joint_names = self._node.joint_names

        package_path = get_package_share_directory(self._node.gait_package)
        self._gait_directory = os.path.join(package_path, self._node.directory_name)
        self._default_yaml = os.path.join(self._gait_directory, "default.yaml")
        self.loaded_gaits = {}
        self._named_positions = {}
        self.gait_positions = {}

        self.gait_duration = 3.0
        self.num_trajectory_points = 4

        self._load_gaits()

    @property
    def get_joint_names(self) -> List[str]:
        """Return a list containing joint names."""
        return self._actuating_joint_names

    @property
    def gaits(self) -> dict:
        """Return a dictionary containing the loaded gaits."""
        return self.loaded_gaits

    @property
    def positions(self) -> Dict[EdgePosition, str]:
        """Returns the named idle positions."""
        return self._named_positions

    def _load_gaits(self) -> None:
        """Load all gaits."""
        self._load_named_positions()
        self._load_home_gaits()
        self._load_sit_and_stand_gaits()

    def _load_named_positions(self) -> None:
        """Load the named positions from default.yaml."""
        with open(self._default_yaml, "r") as default_yaml_file:
            default_config = yaml.load(default_yaml_file, Loader=yaml.SafeLoader)

        self._gait_version_map = default_config["gaits"]
        self._node.get_logger().info("gait version map is: " + str(self._gait_version_map))

        for position_name, position_values in default_config["positions"].items():
            edge_position = StaticEdgePosition(
                [position_values["joints"][joint_name] for joint_name in self._actuating_joint_names]
            )
            self.gait_positions[position_name] = edge_position
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
            home_gait = HomeGait(name, position, "", self._actuating_joint_names)
            self.gaits[home_gait.name] = home_gait

    def _load_sit_and_stand_gaits(self) -> None:
        """Loads the sit and stand gaits."""
        for gait in self._gait_version_map:
            self._node.get_logger().info("gait is: " + str(gait))
            self.loaded_gaits[gait] = SetpointsGait.from_file(gait, self._gait_directory, self._gait_version_map)

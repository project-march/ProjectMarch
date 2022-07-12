"""Author: ???."""

import os
from typing import Dict, Optional, Union

import yaml

from march_utility.gait.edge_position import (
    StaticEdgePosition,
    DynamicEdgePosition,
    EdgePosition,
)
from rclpy.node import Node
from urdf_parser_py import urdf

from march_utility.exceptions.gait_exceptions import (
    GaitNameNotFoundError,
    NonValidGaitContentError,
    SubgaitNameNotFoundError,
)

from .subgait import Subgait
from .subgait_graph import SubgaitGraph

ALLOWED_ERROR_ENDPOINTS = 0.001


class Gait:
    """It verifies the gait at initialization.

    Args:
        gait_name (str): Name of the gait.
        subgaits (dict): Mapping of names to subgait instances.
        graph (SubgaitGraph): Mapping of subgait names transitions.

    Attributes:
        gait_name (str): Name of the gait.
        subgaits (dict): Mapping of names to subgait instances.
        graph (SubgaitGraph): Mapping of subgait names transitions.
        _starting_position (StaticEdgePosition): The starting position of the gait.
        _final_position (StaticEdgePosition): The end position of the gait.
    """

    def __init__(self, gait_name: str, subgaits: dict, graph: SubgaitGraph):
        self.gait_name = gait_name
        self.subgaits = subgaits
        self.graph = graph
        self._starting_position = StaticEdgePosition(self.subgaits[self.graph.start_subgaits()[0]].starting_position)
        self._final_position = StaticEdgePosition(self.subgaits[self.graph.end_subgaits()[0]].final_position)
        self._validate_trajectory_transition()

    @classmethod
    def from_file(
        cls,
        gait_name: str,
        gait_directory: str,
        robot: urdf.Robot,
        gait_version_map: dict,
    ):
        """Extract the data from the .gait file.

        Args:
          gait_name: name of the gait to unpack
          gait_directory: path of the directory where the .gait file is located
          robot: the robot corresponding to the given .gait file
          gait_version_map: The parsed yaml file which states the version of the subgaits
        Returns:
            Gait: If the data in the files is validated a gait object is returned
        """
        gait_folder = gait_name
        gait_path = os.path.join(gait_directory, gait_folder, gait_name + ".gait")
        with open(gait_path, "r") as gait_file:
            gait_dictionary = yaml.load(gait_file, Loader=yaml.SafeLoader)

        return cls.from_dict(robot, gait_dictionary, gait_directory, gait_version_map)

    @classmethod
    def from_dict(
        cls,
        robot: urdf.Robot,
        gait_dictionary: Dict[str, Union[str, Dict[str, str]]],  # noqa TAE002 too complex annotation
        gait_directory: str,
        gait_version_map: dict,
    ):
        """Create a new gait object using the .gait and .subgait files.

        Args:
          robot: the robot corresponding to the given .gait file
          gait_dictionary: the information of the .gait file as a dictionary
          gait_directory: path of the directory where the .gait file is located
          gait_version_map: The parsed yaml file which states the version of the subgaits

        Returns:
            Gait: If the data in the files is validated a gait object is returned
        """
        gait_name = gait_dictionary["name"]
        subgaits = gait_dictionary["subgaits"]

        graph = SubgaitGraph(subgaits)
        subgaits = {
            name: cls.load_subgait(robot, gait_directory, gait_name, name, gait_version_map)
            for name in subgaits
            if name not in ("start", "end")
        }
        return cls(gait_name, subgaits, graph)

    @property
    def starting_position(self):
        """StaticEdgePosition. The starting position of the Gait."""
        return self._starting_position

    @property
    def final_position(self):
        """StaticEdgePosition. The end position of the Gait."""
        return self._final_position

    @staticmethod
    def load_subgait(
        robot: urdf.Robot,
        gait_directory: str,
        gait_name: str,
        subgait_name: str,
        gait_version_map: dict,
    ) -> Subgait:
        """Read the .subgait file and extract the data.

        Args:
          robot: the robot corresponding to the given .gait file
          gait_directory: path of the directory where the .gait file is located
          gait_name: the name of the gait where the subgait belongs to
          subgait_name: the name of the subgait to load
          gait_version_map: the parsed yaml file which states the version of the subgaits

        Returns:
          Gait if gait and subgait names are valid return populated Gait object
        """
        if gait_name not in gait_version_map:
            raise GaitNameNotFoundError(gait_name)
        if subgait_name not in gait_version_map[gait_name]:
            raise SubgaitNameNotFoundError(subgait_name, gait_name)

        version = gait_version_map[gait_name][subgait_name]
        return Subgait.from_name_and_version(robot, gait_directory, gait_name, subgait_name, version)

    def _validate_trajectory_transition(self):
        """Compares and validates the trajectory end and start points."""
        for from_subgait_name, to_subgait_name in self.graph:
            if len({from_subgait_name, to_subgait_name} & {self.graph.START, self.graph.END}) > 0:
                continue

            from_subgait = self.subgaits[from_subgait_name]
            to_subgait = self.subgaits[to_subgait_name]

            if not from_subgait.validate_subgait_transition(to_subgait):
                raise NonValidGaitContentError(
                    msg=f"Gait {self.gait_name} with end setpoint of subgait "
                    f"{from_subgait.subgait_name} to subgait "
                    f"{to_subgait.subgait_name} does not match"
                )

    def _validate_new_edge_position(
        self,
        old_edge_position: EdgePosition,
        new_edge_position_values: Dict[str, float],
    ) -> EdgePosition:
        """Validate that the edge position has made an acceptable change.

        Note:
            StaticEdgePosition -> Should remain the same, values and type.
            UnknownEdgePosition -> Should never be updated.
            DynamicEdgePosition -> Values can change.

        Args:
          old_edge_position: The position the gait has now.
          new_edge_position: The new edge position that should be validated.
          old_edge_position (EdgePosition):
          new_edge_position_values (Dict[str, float]):

        Returns:
            EdgePosition
        """
        if isinstance(old_edge_position, StaticEdgePosition):
            if old_edge_position != StaticEdgePosition(new_edge_position_values):
                raise NonValidGaitContentError(
                    msg="The edge position of new version does not match to the static old version"
                )
            else:
                return StaticEdgePosition(new_edge_position_values)
        elif isinstance(old_edge_position, DynamicEdgePosition):
            return DynamicEdgePosition(new_edge_position_values)
        else:
            raise NonValidGaitContentError(msg="Gaits with unknown edge positions should not be updated")

    def _validate_and_set_new_edge_positions(self, new_subgaits: Dict[str, Subgait]) -> bool:
        """Validate that both the starting position and the final position changed in a valid way.

        Args:
            new_subgaits (Dict[str, Subgait]): the new subgaits that will be used
        Returns:
            bool: True if edge positions are validated
        Raises:
            NonValidGaitContentError: raised if edge position are not valid
        """
        new_starting_position = None
        new_final_position = None
        for from_subgait_name, to_subgait_name in self.graph:
            if from_subgait_name == self.graph.START:
                new_subgait = new_subgaits[to_subgait_name]
                new_starting_position_values = new_subgait.starting_position
                try:
                    new_starting_position = self._validate_new_edge_position(
                        self.starting_position,
                        new_starting_position_values,
                    )
                except NonValidGaitContentError as e:
                    raise e

            elif to_subgait_name == self.graph.END:
                new_subgait = new_subgaits[from_subgait_name]
                new_final_position_values = new_subgait.final_position
                try:
                    new_final_position = self._validate_new_edge_position(
                        self.final_position,
                        new_final_position_values,
                    )
                except NonValidGaitContentError as e:
                    raise e
        self.set_edge_positions(new_starting_position, new_final_position)
        return True

    def set_edge_positions(self, starting_position: EdgePosition, final_position: EdgePosition):
        """Set the new edge positions.

        Overrides from the setpoints gait, which does not store the starting or final position.

        Args:
            starting_position (EdgePosition): the new starting position
            final_position (EdgePosition): the new final position
        """
        self._starting_position = starting_position
        self._final_position = final_position

    def set_subgaits(self, new_subgaits: Dict[str, Subgait], node: Optional[Node] = None):
        """Update the subgaits of the gaits, and validate that the edges changes acceptably.

        Also make sure that all transitions between the subgaits match.

        Args:
            new_subgaits (Dict[str, Subgait]): the dictionary with the new subgaits to use
            node (:obj: Node, optional): A node to use for logging the warnings
        Raises:
            NonValidGaitContentError: raised if new subgaits are invalid
        """
        try:
            if self._validate_and_set_new_edge_positions(new_subgaits):
                self.subgaits = new_subgaits
                self._validate_trajectory_transition()
                return True
        except NonValidGaitContentError as e:
            if node is not None:
                node.get_logger().warn(f"New subgaits were invalid: {e}")
            raise e

    def set_subgait_versions(
        self,
        robot: urdf.Robot,
        gait_directory: str,
        version_map: Dict[str, str],
    ):
        """Updates the given subgait versions and verifies transitions.

        Args:
            robot (urdf.Robot): URDF matching subgaits
            gait_directory (str): path to the gait directory
            version_map (Dict[str, str): mapping subgait names to versions
        Raises:
            NonValidGaitContentError: raised if new subgaits cannot be set
        """
        new_subgaits = self.subgaits.copy()
        for subgait_name, version in version_map.items():
            if subgait_name not in self.subgaits:
                raise SubgaitNameNotFoundError(subgait_name, self.gait_name)
            new_subgaits[subgait_name] = Subgait.from_name_and_version(
                robot, gait_directory, self.gait_name, subgait_name, version
            )

        try:
            self.set_subgaits(new_subgaits)
        except NonValidGaitContentError as e:
            raise e

    def __getitem__(self, name: str) -> Subgait:
        """Returns a subgait from the loaded subgaits.

        Args:
            name (str): name of the subgait
        Returns:
            Subgait: returns subgait with name from the loaded subgaits
        """
        return self.subgaits.get(name)

import os

import yaml
from typing import Dict
from march_utility.gait.edge_position import StaticEdgePosition, DynamicEdgePosition, \
    UnknownEdgePosition, EdgePosition
from urdf_parser_py import urdf

from march_utility.exceptions.gait_exceptions import (
    GaitNameNotFound,
    NonValidGaitContent,
    SubgaitNameNotFound,
)

from .subgait import Subgait
from .subgait_graph import SubgaitGraph

ALLOWED_ERROR_ENDPOINTS = 0.001


class Gait:
    """Base class for a generated gait."""

    def __init__(self, gait_name: str, subgaits: dict, graph: SubgaitGraph):
        """Initializes and verifies the gait.

        :param str gait_name: Name of the gait
        :param dict subgaits: Mapping of names to subgait instances
        :param SubgaitGraph graph: Mapping of subgait names transitions
        """
        self.gait_name = gait_name
        self.subgaits = subgaits
        self.graph = graph

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

        :param gait_name:
            name of the gait to unpack
        :param gait_directory:
            path of the directory where the .gait file is located
        :param robot:
            the robot corresponding to the given .gait file
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits
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
        gait_dictionary: dict,
        gait_directory: str,
        gait_version_map: dict,
    ):
        """Create a new gait object using the .gait and .subgait files.

        :param robot:
            the robot corresponding to the given .gait file
        :param gait_dictionary:
            the information of the .gait file as a dictionary
        :param gait_directory:
            path of the directory where the .gait file is located
        :param gait_version_map:
            The parsed yaml file which states the version of the subgaits

        :return:
            If the data in the files is validated a gait object is returned
        """
        gait_name = gait_dictionary["name"]
        subgaits = gait_dictionary["subgaits"]

        graph = SubgaitGraph(subgaits)
        subgaits = {
            name: cls.load_subgait(
                robot, gait_directory, gait_name, name, gait_version_map
            )
            for name in subgaits
            if name not in ("start", "end")
        }
        return cls(gait_name, subgaits, graph)


    @property
    def starting_position(self) -> EdgePosition:
        """Returns the starting position of all joints."""
        return None

    @property
    def final_position(self) -> EdgePosition:
        """Returns the position of all the joints after the gait has ended."""
        return None


    @staticmethod
    def load_subgait(
        robot: urdf.Robot,
        gait_directory: str,
        gait_name: str,
        subgait_name: str,
        gait_version_map: dict,
    ) -> Subgait:
        """Read the .subgait file and extract the data.
        :param robot: the robot corresponding to the given .gait file
        :param gait_directory: path of the directory where the .gait file is located
        :param gait_name: the name of the gait where the subgait belongs to
        :param subgait_name: the name of the subgait to load
        :param gait_version_map: the parsed yaml file which states the version of
        the subgaits
        :return: Gait if gait and subgait names are valid return populated Gait object
        """
        if gait_name not in gait_version_map:
            raise GaitNameNotFound(gait_name)
        if subgait_name not in gait_version_map[gait_name]:
            raise SubgaitNameNotFound(subgait_name, gait_name)

        version = gait_version_map[gait_name][subgait_name]
        return Subgait.from_name_and_version(
            robot, gait_directory, gait_name, subgait_name, version
        )

    def _validate_trajectory_transition(self):
        """Compares and validates the trajectory end and start points."""
        for from_subgait_name, to_subgait_name in self.graph:
            if (
                len(
                    {from_subgait_name, to_subgait_name}
                    & {self.graph.START, self.graph.END}
                )
                > 0
            ):
                continue

            from_subgait = self.subgaits[from_subgait_name]
            to_subgait = self.subgaits[to_subgait_name]

            if not from_subgait.validate_subgait_transition(to_subgait):
                raise NonValidGaitContent(
                    msg=f"Gait {self.gait_name} with end setpoint of subgait "
                        f"{from_subgait.subgait_name} to subgait "
                        f"{to_subgait.subgait_name} does not match"
                )

    def _validate_new_edge_position(self, gait_edge_position: EdgePosition,
                                    new_subgait_position: Dict[str, float],
                                        subgait_name, node=None):
        if node is not None:
            node.get_logger().info(f"Validate type {type(gait_edge_position)}, "
                                   f"subgait name is {subgait_name}")
        if isinstance(gait_edge_position, StaticEdgePosition):
            # if node is not None:
            #     node.get_logger().info(f"Is static: {new_subgait_position}")
            #     node.get_logger().info(f"{gait_edge_position.values}")
            #     node.get_logger().info(f"Validate static, looking at difference "
            #                            f"between "
            #                            f"{gait_edge_position.values} "
            #                            f"and "
            #                            f"{StaticEdgePosition(new_subgait_position).values}")
            if gait_edge_position != StaticEdgePosition(new_subgait_position):
                node.get_logger().warn(f"The static edge position of new version does "
                                       f"not match the old edge position.")
                raise NonValidGaitContent(
                    msg=f"The edge position of new version does not match"
                )
            return
        elif isinstance(gait_edge_position, UnknownEdgePosition):
            raise NonValidGaitContent(
                msg=f"Gaits with unknown edge positions should not be updated"
            )
        elif isinstance(gait_edge_position, DynamicEdgePosition):
            return

    def _validate_new_edge_positions(self, new_subgaits, node=None):
        if node is not None:
            node.get_logger().info("Validate new edge Positions")
        for from_subgait_name, to_subgait_name in self.graph:
            if from_subgait_name == self.graph.START:
                new_subgait = new_subgaits[to_subgait_name]
                new_starting_position = new_subgait.starting_position
                if node is not None:
                    node.get_logger().info(f"Validating starting pos changed to "
                                           f"{new_subgait.starting_position}")
                self._validate_new_edge_position(
                    self.starting_position,
                    new_starting_position,
                    to_subgait_name,
                    node
                )

            elif to_subgait_name == self.graph.END:
                new_subgait = new_subgaits[from_subgait_name]
                new_final_position = new_subgait.final_position
                if node is not None:
                    node.get_logger().info(f"Validating final pos changed to "
                                           f"{new_subgait.final_position}")
                self._validate_new_edge_position(
                    self.final_position,
                    new_final_position,
                    from_subgait_name,
                    node
                )

        if node is not None:
            node.get_logger().info('Validated all edge positions')

    def set_subgaits(self, new_subgaits: Dict[str, Subgait], node=None):
        if node is not None:
            node.get_logger().info('Updating subgaits')
        self._validate_new_edge_positions(new_subgaits, node)
        if node is not None:
            node.get_logger().info('Validated aledge positions')
        self.subgaits.update(new_subgaits)
        if node is not None:
            node.get_logger().info('Updated')
        self._validate_trajectory_transition()
        if node is not None:
            node.get_logger().info('Validated transitions')

    def set_subgait_versions(
        self, robot: urdf.Robot, gait_directory: str, version_map: dict
    ):
        """Updates the given subgait versions and verifies transitions.

        :param robot: URDF matching subgaits
        :param str gait_directory: path to the gait directory
        :param dict version_map: Mapping subgait names to versions
        """
        new_subgaits = {}
        for subgait_name, version in version_map.items():
            if subgait_name not in self.subgaits:
                raise SubgaitNameNotFound(subgait_name, self.gait_name)
            new_subgaits[subgait_name] = Subgait.from_name_and_version(
                robot, gait_directory, self.gait_name, subgait_name, version
            )
        self.set_subgaits(new_subgaits)

    def __getitem__(self, name: str):
        """Returns a subgait from the loaded subgaits."""
        return self.subgaits.get(name)

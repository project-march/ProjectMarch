"""Author: Unknown."""

import os
from typing import Dict, Union

import yaml
from march_gait_selection.gaits.setpoints_gait import SetpointsGait
from march_utility.gait.edge_position import DynamicEdgePosition, StaticEdgePosition
from march_utility.gait.subgait import Subgait
from march_utility.gait.subgait_graph import SubgaitGraph
from urdf_parser_py import urdf


class DynamicEdgeSetpointsGait(SetpointsGait):
    """The standard gait built up from setpoints, but with dynamic edge positions.

    This is mostly useful for debugging realsense gaits and testing this without restarting the state machine.

    Args:
        gait_name (str): name of the gait
        subgaits (dict): Mapping of names to subgait instances
        graph (SubgaitGraph): Mapping of subgait names to transitions
        start_is_dynamic (bool): True if start is dynamic, else False
        final_is_dynamic (bool): True if final is dynamic, else False
    """

    def __init__(
        self,
        gait_name: str,
        subgaits: Dict[str, Subgait],
        graph: SubgaitGraph,
        start_is_dynamic: bool,
        final_is_dynamic: bool,
    ):
        super(DynamicEdgeSetpointsGait, self).__init__(gait_name, subgaits, graph)
        if start_is_dynamic:
            self._starting_position = DynamicEdgePosition(
                self.subgaits[self.graph.start_subgaits()[0]].starting_position
            )
        else:
            self._starting_position = StaticEdgePosition(
                self.subgaits[self.graph.start_subgaits()[0]].starting_position
            )
        if final_is_dynamic:
            self._final_position = DynamicEdgePosition(self.subgaits[self.graph.end_subgaits()[0]].final_position)
        else:
            self._final_position = StaticEdgePosition(self.subgaits[self.graph.end_subgaits()[0]].final_position)

    @classmethod
    def dynamic_from_file(
        cls,
        gait_name: str,
        gait_directory: str,
        robot: urdf.Robot,
        gait_version_map: Dict[str, str],
        start_is_dynamic: bool,
        final_is_dynamic: bool,
    ):
        """Extract the data from the .gait file.

        Args:
            gait_name (str): name of the gait to unpack
            gait_directory (str): path of the directory where the .gait file is located
            robot (urdf.Robot): the robot corresponding to the given .gait file
            gait_version_map (dict[str, str]): The parsed yaml file which states the version of the subgaits
            start_is_dynamic (bool): True if start is dynamic, else False
            final_is_dynamic (bool): True if final is dynamic, else False
        """
        gait_folder = gait_name
        gait_path = os.path.join(gait_directory, gait_folder, gait_name + ".gait")
        with open(gait_path, "r") as gait_file:
            gait_dictionary = yaml.load(gait_file, Loader=yaml.SafeLoader)

        return cls.dynamic_from_dict(
            robot,
            gait_dictionary,
            gait_directory,
            gait_version_map,
            start_is_dynamic,
            final_is_dynamic,
        )

    @classmethod
    def dynamic_from_dict(
        cls,
        robot: urdf.Robot,
        gait_dictionary: Dict[str, Union[str, Dict[str, str]]], # noqa TAE002 suppress to complex expression
        gait_directory: str,
        gait_version_map: dict,
        start_is_dynamic: bool,
        final_is_dynamic: bool,
    ):
        """Create a new gait object using the .gait and .subgait files.

        Args:
            robot (urdf.Robot): the robot corresponding to the given .gait file
            gait_dictionary (dict): the information of the .gait file as a dictionary
            gait_directory (str): path of the directory where the .gait file is located
            gait_version_map (dict[str, str]): The parsed yaml file which states the version of the subgaits
            start_is_dynamic (bool): True if start is dynamic, else False
            final_is_dynamic (bool): True if final is dynamic, else False
        Returns:
            If the data in the files is validated a gait object is returned
        """
        gait_name = gait_dictionary["name"]
        subgaits = gait_dictionary["subgaits"]

        graph = SubgaitGraph(subgaits)
        subgaits = {
            name: cls.load_subgait(robot, gait_directory, gait_name, name, gait_version_map)
            for name in subgaits
            if name not in ("start", "end")
        }
        return cls(gait_name, subgaits, graph, start_is_dynamic, final_is_dynamic)

    @property
    def starting_position(self):
        """Returns the starting position of the gait."""
        return self._starting_position

    @property
    def final_position(self):
        """Returns the final position of the gait."""
        return self._final_position

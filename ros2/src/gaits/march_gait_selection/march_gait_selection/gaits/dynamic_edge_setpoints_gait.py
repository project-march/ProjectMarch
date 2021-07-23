import os

import yaml
from march_gait_selection.gaits.setpoints_gait import SetpointsGait
from march_utility.gait.edge_position import DynamicEdgePosition, StaticEdgePosition
from march_utility.gait.subgait_graph import SubgaitGraph
from urdf_parser_py import urdf


class DynamicEdgeSetpointsGait(SetpointsGait):
    """The standard gait built up from setpoints, but with dynamic edge positions.
    This is mostly useful for debugging realsense gaits and testing this without
    restarting the state machine."""

    def __init__(self, gait_name, subgaits, graph, start_is_dynamic, final_is_dynamic):
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
            self._final_position = DynamicEdgePosition(
                self.subgaits[self.graph.end_subgaits()[0]].final_position
            )
        else:
            self._final_position = StaticEdgePosition(
                self.subgaits[self.graph.end_subgaits()[0]].final_position
            )


    @classmethod
    def dynamic_from_file(
            cls,
            gait_name: str,
            gait_directory: str,
            robot: urdf.Robot,
            gait_version_map: dict,
            start_is_dynamic: bool,
            final_is_dynamic: bool
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

        return cls.dynamic_from_dict(robot, gait_dictionary, gait_directory,
                             gait_version_map, start_is_dynamic, final_is_dynamic)

    @classmethod
    def dynamic_from_dict(
            cls,
            robot: urdf.Robot,
            gait_dictionary: dict,
            gait_directory: str,
            gait_version_map: dict,
            start_is_dynamic: bool,
            final_is_dynamic: bool
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
        return cls(gait_name, subgaits, graph, start_is_dynamic,  final_is_dynamic)

    @property
    def starting_position(self):
        return self._starting_position

    @property
    def final_position(self):
        return self._final_position

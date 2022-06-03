"""Author: Marten Haitjema."""

import os
from typing import List, Optional

import yaml
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from urdf_parser_py import urdf

from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_walk import DynamicGaitWalk
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_step_and_close import DynamicGaitStepAndClose
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_step import DynamicGaitStep
from march_gait_selection.dynamic_interpolation.gaits.dynamic_gait_close import DynamicGaitClose
from march_gait_selection.dynamic_interpolation.cybathlon_obstacle_gaits.stepping_stones_step_and_close import (
    SteppingStonesStepAndClose,
)
from march_gait_selection.dynamic_interpolation.cybathlon_obstacle_gaits.dynamic_gait_step_and_hold import (
    DynamicGaitStepAndHold,
)
from march_gait_selection.dynamic_interpolation.fixed_gaits.fixed_gait_walk import FixedGaitWalk
from march_gait_selection.dynamic_interpolation.fixed_gaits.fixed_gait_step_and_close import FixedGaitStepAndClose
from march_gait_selection.dynamic_interpolation.fixed_gaits.fixed_gait_step import FixedGaitStep

from march_gait_selection.gaits.home_gait import HomeGait
from march_gait_selection.gaits.setpoints_gait import SetpointsGait
from march_utility.exceptions.gait_exceptions import NonValidGaitContentError
from march_utility.gait.edge_position import UnknownEdgePosition
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import get_joint_names_from_robot

NODE_NAME = "gait_selection"
UNKNOWN = "unknown"


class GaitLoader:
    """Base class for the gait selection module."""

    def __init__(
        self,
        node: Node,
        robot: Optional[urdf.Robot] = None,
    ):
        self._node = node
        self._robot = robot
        self._logger = Logger(self._node, __class__.__name__)
        self._joint_names = sorted(get_joint_names_from_robot(self._robot))

        package_path = get_package_share_directory(self._node.gait_package)
        self._gait_directory = os.path.join(package_path, self._node.directory_name)
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

    def _load_gaits(self) -> None:
        """Load all gaits."""
        self._load_named_positions()
        self._load_dynamic_gaits()
        self._load_home_gaits()
        self._load_sit_and_stand_gaits()

    def _load_dynamic_gaits(self) -> None:
        """Load the dynamic gait classes."""
        dynamic_gaits = [
            DynamicGaitWalk(self._node, self.positions),
            DynamicGaitStep(self._node, self.positions),
            DynamicGaitStepAndClose(self._node, self.positions),
            DynamicGaitClose(self._node, self.positions),
            FixedGaitWalk(self._node, self.positions),
            FixedGaitStepAndClose(self._node, self.positions),
            FixedGaitStep(self._node, self.positions),
        ]

        if self._node.add_cybathlon_gaits:
            dynamic_gaits.append(DynamicGaitStepAndHold(self._node, self.positions))
            dynamic_gaits.append(SteppingStonesStepAndClose(self._node, self.positions))

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

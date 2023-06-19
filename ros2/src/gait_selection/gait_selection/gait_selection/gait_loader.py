"""Author: Marco Bak MVIII."""

import os
import yaml

from typing import List
from ament_index_python import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GaitLoader:
    """Class that returns a dictionary of all loaded gaits and positions.

    Args:
        node (rclpy.Node): the gait node
    Attributes:
        _node (rclpy.Node): the gait node
        _default_yaml (str): path to the yaml that contains the named positions
    """

    def __init__(
            self,
            node: Node,
    ):
        """Init the gait loader for the gait_selection."""
        self._node = node

        self._node.declare_parameter('robot', 'march8')
        robot = self._node.get_parameter('robot').get_parameter_value().string_value

        robot_path = get_package_share_directory('march_hardware_builder') + '/robots/' + robot + '.yaml'
        with open(robot_path) as file:
            document = yaml.full_load(file)

        self._actuating_joint_names = []
        robot_name = list(document.keys())[0]
        for joint in document[robot_name]["joints"]:
            for name in joint:
                self._actuating_joint_names.append(name)

        package_path = get_package_share_directory(self._node.gait_package) + "/" + self._node.directory_name
        self._default_yaml = os.path.join(package_path, "gaits.yaml")
        self.loaded_gaits = {}

        self._load_gaits_from_yaml(self._default_yaml)

    @property
    def joint_names(self) -> List[str]:
        """Return a list containing joint names."""
        return self._actuating_joint_names

    @property
    def gaits(self) -> dict:
        """Return a dictionary containing the loaded gaits."""
        return self.loaded_gaits

    def _load_gaits_from_yaml(self, yaml_location):
        with open(yaml_location, "r") as gait_file:
            try:
                gait_yaml = yaml.safe_load(gait_file)
            except yaml.YAMLError as exc:
                self._node.get_logger().error(str(exc))

        gaits = gait_yaml["gaits"].items()
        for gait in gaits:
            trajectory = JointTrajectory()
            gait_name = gait[0]
            gait_points = gait[1]["points"]
            point_amount = gait[1]["amount_of_points"]
            trajectory.joint_names = gait[1]["joint_names"]
            for i in range(point_amount):
                point = JointTrajectoryPoint()
                point.time_from_start = Duration(seconds=gait_points["time_from_start"][i][0],
                                                 nanoseconds=gait_points["time_from_start"][i][1]).to_msg()
                point.positions = gait_points["positions"][i]
                point.velocities = gait_points["velocities"][i]
                point.accelerations = []
                point.effort = []
                trajectory.points.append(point)
            self.loaded_gaits[gait_name] = trajectory

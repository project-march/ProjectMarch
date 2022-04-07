"""Author: Marten Haitjema, MVII"""

import numpy as np

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import (
    DynamicJointTrajectory,
)
from march_utility.gait.limits import Limits
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import get_position_from_yaml
from march_utility.utilities.logger import Logger
from march_goniometric_ik_solver.ik_solver import Pose

from trajectory_msgs import msg as trajectory_msg
from march_shared_msgs.msg import FootPosition

from typing import List, Optional
from enum import IntEnum

EXTRA_ANKLE_SETPOINT_INDEX = 1
INTERPOLATION_POINTS = 30


class SetpointTime(IntEnum):
    START_INDEX = 0
    PUSH_OFF_INDEX = 1
    MIDDLE_POINT_INDEX = 2
    END_POINT_INDEX = 3


class DynamicSubgait:
    """Creates joint trajectories based on the desired foot location.

    :param gait_selection_node: The gait_selection node
    :type gait_selection: Node
    :param starting_position: The first setpoint of the subgait, usually the last setpoint of the previous subgait.
    :type starting_position: dict
    :param subgait_id: Whether it is a left_swing or right_swing.
    :type subgait_id: str
    :param joint_names: Names of the joints
    :type joint_names: list
    :param location: Desired foot position
    :type location: Point
    :param joint_soft_limits: list containing soft limits in alphabetical order
    :type joint_soft_limits: List[Limits]
    :param start: whether it is an open gait or not
    :type start: bool
    :param stop: whether it is a close gait or not
    :type stop: bool
    """

    def __init__(
        self,
        gait_selection_node: Node,
        starting_position: dict,
        subgait_id: str,
        joint_names: List[str],
        location: FootPosition,
        joint_soft_limits: List[Limits],
        start: bool,
        stop: bool,
    ):
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self._get_parameters(gait_selection_node)

        self.starting_position = starting_position
        self.location = location.processed_point
        self.actuating_joint_names = joint_names
        self.all_joint_names = list(starting_position.keys())
        self.subgait_id = subgait_id
        self.joint_soft_limits = joint_soft_limits

        self.time = [
            0,
            self.push_off_fraction * location.duration,
            self.middle_point_fraction * location.duration,
            location.duration,
        ]

        self.starting_position_dict = self._from_list_to_setpoint(
            self.actuating_joint_names, list(self.starting_position.values()), None, self.time[SetpointTime.START_INDEX]
        )

        self.start = start
        self.stop = stop

    def get_joint_trajectory_msg(self) -> trajectory_msg.JointTrajectory:
        """Return a joint_trajectory_msg containing the interpolated
        trajectories for each joint

        :returns: A joint_trajectory_msg
        :rtype: joint_trajectory_msg
        """
        self.pose = Pose(self.all_joint_names, list(self.starting_position.values()))

        self._solve_middle_setpoint()
        self._solve_desired_setpoint()
        self._get_extra_ankle_setpoint()

        # Create joint_trajectory_msg
        self._to_joint_trajectory_class()
        joint_trajectory_msg = trajectory_msg.JointTrajectory()
        joint_trajectory_msg.joint_names = self.actuating_joint_names

        timestamps = np.linspace(self.time[0], self.time[-1], INTERPOLATION_POINTS)
        for timestamp in timestamps:
            joint_trajectory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = Duration(timestamp).to_msg()

            for joint_index, joint_trajectory in enumerate(self.joint_trajectory_list):
                interpolated_setpoint = joint_trajectory.get_interpolated_setpoint(timestamp)

                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)
                self._check_joint_limits(joint_index, joint_trajectory_point)

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    def _get_extra_ankle_setpoint(self) -> Setpoint:
        """Returns an extra setpoint for the swing leg ankle
        that can be used to create a push off.

        :returns: An extra setpoint for the swing leg ankle
        :rtype: Setpoint
        """
        return Setpoint(
            Duration(self.time[SetpointTime.PUSH_OFF_INDEX]),
            self.push_off_position,
            0.0,
        )

    def _solve_middle_setpoint(self) -> None:
        """Calls IK solver to compute the joint angles needed for the middle setpoint

        :returns: A setpoint_dict for the middle position.
        :rtype: dict
        """
        middle_position = self.pose.solve_mid_position(
            self.location.x,
            self.location.y,
            self.location.z,
            self.middle_point_fraction,
            self.middle_point_height,
            self.subgait_id,
        )

        self.middle_setpoint_dict = self._from_list_to_setpoint(
            self.all_joint_names,
            middle_position,
            None,
            self.time[SetpointTime.MIDDLE_POINT_INDEX],
        )

    def _solve_desired_setpoint(self) -> None:
        """Calls IK solver to compute the joint angles needed for the
        desired x and y coordinate"""
        if self.stop:
            self.desired_position = list(get_position_from_yaml("stand").values())
        else:
            self.desired_position = self.pose.solve_end_position(
                self.location.x, self.location.y, self.location.z, self.subgait_id
            )

        self.desired_setpoint_dict = self._from_list_to_setpoint(
            self.all_joint_names,
            self.desired_position,
            None,
            self.time[SetpointTime.END_POINT_INDEX],
        )

    def _to_joint_trajectory_class(self) -> None:
        """Creates a list of DynamicJointTrajectories for each joint"""
        self.joint_trajectory_list = []
        for name in self.actuating_joint_names:
            setpoint_list = [
                self.starting_position_dict[name],
                self.middle_setpoint_dict[name],
                self.desired_setpoint_dict[name],
            ]

            # Add an extra setpoint to the ankle to create a push off, except for
            # a start gait:
            if not self.start and (
                (name == "right_ankle" and self.subgait_id == "right_swing")
                or (name == "left_ankle" and self.subgait_id == "left_swing")
            ):
                setpoint_list.insert(EXTRA_ANKLE_SETPOINT_INDEX, self._get_extra_ankle_setpoint())

            if name in ["right_ankle", "left_ankle"]:
                self.joint_trajectory_list.append(DynamicJointTrajectory(setpoint_list, interpolate_ankle=True))
            else:
                self.joint_trajectory_list.append(DynamicJointTrajectory(setpoint_list))

    def get_final_position(self) -> dict:
        """Get setpoint_dictionary of the final setpoint.

        :return: The final setpoint of the subgait.
        :rtype: dict
        """
        final_position = {}
        for i, name in enumerate(self.all_joint_names):
            final_position[name] = self.desired_position[i]
        return final_position

    def _from_list_to_setpoint(
        self,
        joint_names: List[str],
        position: List[float],
        velocity: Optional[List[float]],
        time: float,
    ) -> dict:
        """Computes setpoint_dictionary from a list

        :param joint_names: Names of the joints.
        :type joint_names: list
        :param position: Positions for each joint.
        :type position: list
        :param velocity: Optional velocities for each joint. If None, velocity will be set to zero.
        :type velocity: list
        :param time: Time at which the setpoint should be set.
        :type time: float

        :returns: A Setpoint_dict containing time, position and velocity for each joint
        :rtype: dict
        """
        setpoint_dict = {}
        velocity = np.zeros_like(position) if (velocity is None) else velocity

        for i, name in enumerate(joint_names):
            if (name == "right_ankle" and self.subgait_id == "right_swing") or (
                name == "left_ankle" and self.subgait_id == "left_swing"
            ):
                velocity[i] = 0.0

            setpoint_dict.update(
                {
                    joint_names[i]: Setpoint(
                        Duration(time),
                        position[i],
                        velocity[i],
                    )
                }
            )

        return setpoint_dict

    def _get_parameters(self, gait_selection_node: Node) -> None:
        """Gets the dynamic gait parameters from the gait_selection_node

        :param gait_selection_node: the gait selection node
        :type gait_selection_node: Node
        """
        self.middle_point_height = gait_selection_node.middle_point_height
        self.middle_point_fraction = gait_selection_node.middle_point_fraction
        self.push_off_fraction = gait_selection_node.push_off_fraction
        self.push_off_position = gait_selection_node.push_off_position

    def _check_joint_limits(
        self,
        joint_index: int,
        joint_trajectory_point: trajectory_msg.JointTrajectoryPoint,
    ) -> None:
        """Check if values in the joint_trajectory_point are within the soft and
        velocity limits defined in the urdf

        :param joint_index: Index of the joint in the alphabetical joint_names list
        :type joint_index: int
        :param joint_trajectory_point: point in time containing position and velocity
        :type joint_trajectory_point: trajectory_msg.JointTrajectoryPoint
        """
        position = joint_trajectory_point.positions[joint_index]
        velocity = joint_trajectory_point.velocities[joint_index]
        if position > self.joint_soft_limits[joint_index].upper or position < self.joint_soft_limits[joint_index].lower:
            self.logger.info(
                f"DynamicSubgait: {self.actuating_joint_names[joint_index]} will be outside of soft limits, "
                f"position: {position}, soft limits: "
                f"[{self.joint_soft_limits[joint_index].lower}, {self.joint_soft_limits[joint_index].upper}]."
            )
            raise Exception(f"{self.actuating_joint_names[joint_index]} will be outside its soft limits.")

        if abs(velocity) > self.joint_soft_limits[joint_index].velocity:
            self.logger.info(
                f"DynamicSubgait: {self.actuating_joint_names[joint_index]} will be outside of velocity limits, "
                f"velocity: {velocity}, velocity limit: {self.joint_soft_limits[joint_index].velocity}."
            )
            raise Exception(f"{self.actuating_joint_names[joint_index]} will be outside its velocity limits.")

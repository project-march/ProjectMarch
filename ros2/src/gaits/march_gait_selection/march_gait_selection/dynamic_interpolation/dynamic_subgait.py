"""Author: Marten Haitjema, MVII."""

import numpy as np

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import (
    DynamicJointTrajectory,
)
from march_utility.gait.limits import Limits
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.logger import Logger
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
)
from march_goniometric_ik_solver.ik_solver import Pose

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from march_shared_msgs.msg import FootPosition

from typing import List, Dict, Optional
from enum import IntEnum

EXTRA_ANKLE_SETPOINT_INDEX = 1
INTERPOLATION_POINTS = 30


class SetpointTime(IntEnum):
    """Enum for the index of time list."""

    START_INDEX = 0
    PUSH_OFF_INDEX = 1
    MIDDLE_POINT_INDEX = 2
    END_POINT_INDEX = 3


class DynamicSubgait:
    """Creates joint trajectories based on the desired foot location.

    Args:
        gait_selection_node (GaitSelection): The gait selection node
        starting_position (Dict[str, float]): The first setpoint of the subgait, usually the last setpoint
            of the previous subgait.
        subgait_id (str): Whether it is a left_swing or right_swing
        joint_names (List[str]): Names of the joints
        location (Point): Desired location of the foot, given by covid
        joint_soft_limits (List[Limits]): List containing soft limits of joints in alphabetical order
        start (bool): whether it is an open gait or not
        stop (bool): whether it is a close gait or not

    Attributes:
        logger (Logger): used for logging to the terminal
        starting_position (Dict[str, Setpoint]): the first setpoint of the gait
        location (Point): the desired location given by (fake) covid
        joint_names (List[str]): list of joint names
        subgait_id (str): either left_swing or right_swing
        joint_soft_limits (List[Limits]): a list containing the soft limits of each joint
        push_off_fraction (float): fraction of total time of the step at which push off will take place
        middle_point_fraction (float): fraction of total time of the step at which middle point will take place
        start (bool): True if it is an open gait, else False
        stop (bool): True if it is a close gait, else False
        pose (Pose): pose object used to calculate inverse kinematics
    """

    def __init__(
        self,
        gait_selection_node: Node,
        home_stand_position: Dict[str, float],
        starting_position: Dict[str, float],
        subgait_id: str,
        joint_names: List[str],
        location: FootPosition,
        joint_soft_limits: List[Limits],
        start: bool,
        stop: bool,
    ):
        self.logger = Logger(gait_selection_node, __class__.__name__)
        self._get_parameters(gait_selection_node)

        self.home_stand_position = home_stand_position
        self.starting_position = starting_position
        self.location = location.processed_point
        self.actuating_joint_names = joint_names
        self.all_joint_names = list(starting_position.keys())
        self.subgait_id = subgait_id
        self.joint_soft_limits = joint_soft_limits
        self.pose = Pose(self.all_joint_names, list(self.starting_position.values()))

        self.time = [
            0,
            self.push_off_fraction * location.duration,
            self.middle_point_fraction * location.duration,
            location.duration,
        ]

        self.starting_position_dict = self._from_list_to_setpoint(
            self.all_joint_names, list(self.starting_position.values()), None, self.time[SetpointTime.START_INDEX]
        )

        self.start = start
        self.stop = stop

    def get_joint_trajectory_msg(self, push_off: bool) -> JointTrajectory:
        """Return a joint_trajectory_msg containing the interpolated trajectories for each joint.

        Args:
            push_off (bool): True if push off should be present in the gait
        Returns:
            JointTrajectory: message containing interpolated trajectories for each joint
        """
        self._solve_middle_setpoint()
        self._solve_desired_setpoint()

        # Create joint_trajectory_msg
        self._to_joint_trajectory_class(push_off)
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = self.actuating_joint_names

        timestamps = np.linspace(self.time[0], self.time[-1], INTERPOLATION_POINTS)
        for timestamp in timestamps:
            joint_trajectory_point = JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = Duration(timestamp).to_msg()

            for joint_index, joint_trajectory in enumerate(self.joint_trajectory_list):
                interpolated_setpoint = joint_trajectory.get_interpolated_setpoint(timestamp)

                joint_trajectory_point.positions.append(interpolated_setpoint.position)
                joint_trajectory_point.velocities.append(interpolated_setpoint.velocity)
                self._check_joint_limits(joint_index, joint_trajectory_point)

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

    def _get_extra_ankle_setpoint(self) -> Setpoint:
        """Returns an extra setpoint for the swing leg ankle that can be used to create a push off.

        Returns:
            Setpoint: extra setpoint for the swing leg ankle
        """
        return Setpoint(
            Duration(self.time[SetpointTime.PUSH_OFF_INDEX]),
            self.push_off_position,
            0.0,
        )

    def _solve_middle_setpoint(self) -> None:
        """Calls IK solver to compute the joint angles needed for the middle setpoint."""
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
        """Calls IK solver to compute the joint angles needed for the desired x and y coordinate."""
        if self.stop:
            self.desired_position = list(self.home_stand_position.values())
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

    def _to_joint_trajectory_class(self, push_off: bool) -> None:
        """Creates a list of DynamicJointTrajectories for each joint.

        Args:
            push_off (bool): True if push off should be present in the gait
        """
        self.joint_trajectory_list = []
        for name in self.actuating_joint_names:
            setpoint_list = [
                self.starting_position_dict[name],
                self.middle_setpoint_dict[name],
                self.desired_setpoint_dict[name],
            ]

            # Add an extra setpoint to the ankle to create a push off, except for a start gait:
            if (
                push_off
                and not self.start
                and (
                    (name == "right_ankle" and self.subgait_id == "right_swing")
                    or (name == "left_ankle" and self.subgait_id == "left_swing")
                )
            ):
                setpoint_list.insert(EXTRA_ANKLE_SETPOINT_INDEX, self._get_extra_ankle_setpoint())

            if name in ["right_ankle", "left_ankle"] or (
                (name == "right_knee" and self.subgait_id == "left_swing")
                or (name == "left_knee" and self.subgait_id == "right_swing")
            ):
                self.joint_trajectory_list.append(DynamicJointTrajectory(setpoint_list, fixed_midpoint_velocity=True))
            else:
                self.joint_trajectory_list.append(DynamicJointTrajectory(setpoint_list))

    def get_final_position(self) -> Dict[str, float]:
        """Get setpoint_dictionary of the final setpoint.

        Returns:
            dict: The final setpoint of the subgait
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
    ) -> Dict[str, Setpoint]:
        """Computes setpoint_dictionary from a list.

        Args:
            joint_names (:obj: list of :obj: str): Names of the joints
            position (:obj: list of :obj: float): Position for each joint
            velocity (:obj: list of :obj: float, optional): Optional velocity for each joint, default is zero
            time (float): Time at which the setpoint should be set
        Returns:
            dict: A Setpoint_dict containing time, position and velocity for each joint
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
        """Gets the dynamic gait parameters from the gait_selection_node.

        Args:
            gait_selection_node (GaitSelection): the gait selection node
        """
        self.middle_point_height = gait_selection_node.middle_point_height
        self.middle_point_fraction = gait_selection_node.middle_point_fraction
        self.push_off_fraction = gait_selection_node.push_off_fraction
        self.push_off_position = gait_selection_node.push_off_position

    def _check_joint_limits(
        self,
        joint_index: int,
        joint_trajectory_point: JointTrajectoryPoint,
    ) -> None:
        """Check if values in the joint_trajectory_point are within the soft and velocity limits defined in the urdf.

        Args:
            joint_index (int): Index of the joint in the alphabetical joint_names list
            joint_trajectory_point (JointTrajectoryPoint): point in time containing  position and velocity
        """
        position = joint_trajectory_point.positions[joint_index]
        velocity = joint_trajectory_point.velocities[joint_index]
        if position > self.joint_soft_limits[joint_index].upper or position < self.joint_soft_limits[joint_index].lower:
            raise PositionSoftLimitError(
                self.actuating_joint_names[joint_index],
                position,
                self.joint_soft_limits[joint_index].lower,
                self.joint_soft_limits[joint_index].upper,
            )

        if abs(velocity) > self.joint_soft_limits[joint_index].velocity:
            raise VelocitySoftLimitError(
                self.actuating_joint_names[joint_index],
                velocity,
                self.joint_soft_limits[joint_index].velocity,
            )

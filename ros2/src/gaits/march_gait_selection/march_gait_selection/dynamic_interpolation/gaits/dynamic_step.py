"""Author: Marten Haitjema, MVII."""

import numpy as np
import copy

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.gaits.dynamic_joint_trajectory import (
    DynamicJointTrajectory,
)
from march_goniometric_ik_solver.ik_solver_parameters import IKSolverParameters
from march_utility.gait.limits import Limits
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
)
from march_goniometric_ik_solver.ik_solver import Pose

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from march_shared_msgs.msg import FootPosition

from typing import List, Dict, Optional

EXTRA_ANKLE_SETPOINT_INDEX = 1
INTERPOLATION_POINTS = 30


class DynamicStep:
    """Creates joint trajectories based on the desired foot location.

    Args:
        node (Node): The gait node
        starting_position (Dict[str, float]): The first setpoint of the subgait, usually the last setpoint
            of the previous subgait.
        subgait_id (str): Whether it is a left_swing or right_swing
        joint_names (List[str]): Names of the joints
        location (Point): Desired location of the foot, given by covid
        joint_soft_limits (List[Limits]): List containing soft limits of joints in alphabetical order
        start (bool): whether it is an open gait or not
        stop (bool): whether it is a close gait or not
        hold_subgait (bool): whether the subgait is created by the dynamic_setpoint_gait_step_and_hold class

    Attributes:
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
        hold_subgait (bool): whether the subgait is created by the dynamic_setpoint_gait_step_and_hold class
    """

    def __init__(
        self,
        node: Node,
        home_stand_position: Dict[str, float],
        starting_position: Dict[str, float],
        subgait_id: str,
        joint_names: List[str],
        location: FootPosition,
        joint_soft_limits: List[Limits],
        start: bool,
        stop: bool,
        hold_subgait: bool = False,
    ):
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._get_parameters(node)
        self._node = node
        self.home_stand_position = home_stand_position
        self.starting_position = starting_position

        self.location = location.processed_point
        self._duration = location.duration
        self._deviation = location.midpoint_deviation
        self._height = location.relative_midpoint_height

        self.actuating_joint_names = joint_names
        self.all_joint_names = list(starting_position.keys())
        self.subgait_id = subgait_id
        self.joint_soft_limits = joint_soft_limits

        if self.subgait_id == "right_swing":
            self._start_pose = Pose(self._ik_solver_parameters, list(self.starting_position.values()), "right")
            self._end_pose = Pose(self._ik_solver_parameters, list(self.home_stand_position.values()), "left")

        else:
            self._start_pose = Pose(self._ik_solver_parameters, list(self.starting_position.values()), "left")
            self._end_pose = Pose(self._ik_solver_parameters, list(self.home_stand_position.values()), "right")

        self.starting_position_dict = self._from_list_to_setpoint(
            self.all_joint_names,
            list(self.starting_position.values()),
            None,
            0,
        )
        self._logger.warn(
            f"Duration: {self._duration}, "
            f"deviation: {location.midpoint_deviation}, "
            f"height: {location.relative_midpoint_height}"
        )
        self.start = start
        self.stop = stop
        self.hold_subgait = hold_subgait

    def get_joint_trajectory_msg(self, push_off: bool) -> JointTrajectory:
        """Return a joint_trajectory_msg containing the interpolated trajectories for each joint.

        Args:
            push_off (bool): True if push off should be present in the gait
        Returns:
            JointTrajectory: message containing interpolated trajectories for each joint
        """
        desired_position = self._solve_desired_setpoint()
        setpoint_list = [
            self.starting_position_dict,
            self._solve_middle_setpoint(self.middle_point_fraction - self._deviation, self._height),
        ]

        if not self.stop:
            setpoint_list.append(
                self._solve_middle_setpoint(self.middle_point_fraction + self._deviation, self._height)
            )
        else:
            setpoint_list.append(self._solve_middle_setpoint_for_close(self._stop_mid2_fraction, self._stop_mid2_y))

        setpoint_list.append(desired_position)

        # Create joint_trajectory_msg
        self._to_joint_trajectory_class(setpoint_list, push_off)
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.joint_names = self.actuating_joint_names

        timestamps = np.linspace(0, self._duration, INTERPOLATION_POINTS)
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
            Setpoint: Extra setpoint for the swing leg ankle.
        """
        return Setpoint(
            Duration(self.push_off_fraction * self._duration),
            self.push_off_position,
            0.0,
        )

    def _solve_middle_setpoint(self, fraction=None, height=None) -> Dict[str, Setpoint]:
        """Calls IK solver to compute the joint angles needed for the middle setpoint."""
        fraction = self.middle_point_fraction if fraction is None else fraction
        height = self.middle_point_height if height is None else height
        pose = copy.deepcopy(self._start_pose)

        middle_position = pose.solve_mid_position(
            next_pose=self._end_pose,
            frac=fraction,
            ankle_y=height,
            subgait_id=self.subgait_id,
        )

        return self._from_list_to_setpoint(
            self.all_joint_names,
            middle_position,
            None,
            fraction * self._duration,
        )

    def _solve_middle_setpoint_for_close(self, fraction, height) -> Dict[str, Setpoint]:
        pose = copy.deepcopy(self._start_pose)

        middle_position = pose.solve_end_position(
            self._stop_mid2_x,
            height,
            0.51,
            self.subgait_id,
            for_mid_point=True,
        )

        return self._from_list_to_setpoint(self.all_joint_names, middle_position, None, fraction * self._duration)

    def _solve_desired_setpoint(self) -> Dict[str, Setpoint]:
        """Calls IK solver to compute the joint angles needed for the desired x and y coordinate."""
        if self.stop:
            self.desired_position = list(self.home_stand_position.values())
        else:
            self.desired_position = self._end_pose.solve_end_position(
                self.location.x, self.location.y, self.location.z, self.subgait_id
            )

        return self._from_list_to_setpoint(
            self.all_joint_names,
            self.desired_position,
            None,
            self._duration,
        )

    def _to_joint_trajectory_class(self, dict_list: List[Dict[str, Setpoint]], push_off: bool) -> None:
        """Creates a list of DynamicJointTrajectories for each joint.

        Args:
            dict_list ([List[Dict[str, Setpoint]]): list containing a dict containing a joint name and corresponding
                setpoint.
            push_off (bool): True if push off should be present in the gait
        """
        if self.stop and self.hold_subgait:
            dict_list[1] = dict_list[2]

        self.joint_trajectory_list = []
        for name in self.actuating_joint_names:
            setpoint_list = []
            for setpoint_dicts in dict_list:
                setpoint_list.append(setpoint_dicts[name])

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
                or (self.stop and self.hold_subgait)
            ):
                self.joint_trajectory_list.append(DynamicJointTrajectory(setpoint_list, fixed_midpoint_velocity=True))
            else:
                self.joint_trajectory_list.append(
                    DynamicJointTrajectory(
                        setpoint_list,
                        fixed_midpoint_velocity=self._fixed_midpoint_velocity,
                    )
                )

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
        velocity = np.zeros_like(position) if velocity is None else velocity

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

    def _get_parameters(self, node: Node) -> None:
        """Gets the dynamic gait parameters from the node.

        Args:
            node (Node): the gait selection node
        """
        self.middle_point_height = node.middle_point_height
        self.middle_point_fraction = node.middle_point_fraction
        self.push_off_fraction = node.push_off_fraction
        self.push_off_position = node.push_off_position
        self._fixed_midpoint_velocity = node.fixed_midpoint_velocity
        self._stop_mid2_fraction = node.stop_mid2_fraction
        self._stop_mid2_x = node.stop_mid2_x
        self._stop_mid2_y = node.stop_mid2_y
        self._ik_solver_parameters = IKSolverParameters(
            node.ankle_buffer,
            node.hip_buffer,
            node.default_knee_bend,
            node.hip_x_fraction,
            node.upper_body_front_rotation,
            node.dorsiflexion_at_end_position,
            node.hip_swing,
            node.hip_swing_fraction,
            node.middle_point_fraction,
            node.base_number,
        )

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

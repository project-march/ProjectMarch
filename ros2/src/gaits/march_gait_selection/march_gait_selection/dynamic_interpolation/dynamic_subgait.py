import numpy as np

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import (
    DynamicJointTrajectory,
)
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import get_position_from_yaml
from march_goniometric_ik_solver.ik_solver import Pose

from trajectory_msgs import msg as trajectory_msg
from geometry_msgs.msg import Point

from typing import List

EXTRA_ANKLE_SETPOINT_INDEX = 1


class DynamicSubgait:
    """Creates joint trajectories based on the desired foot location.

    :param duration: Duration of the subgait.
    :type duration: float
    :param middle_point_fraction: Fraction of the subgait at which the middle setpoint will be set.
    :type middle_point_fraction: float
    :param middle_point_height: Height of the middle setpoint.
    :type middle_point_height: float
    :param starting_position: The first setpoint of the subgait, usually the last setpoint of the previous subgait.
    :type starting_position: dict
    :param subgait_id: Whether it is a left_swing or right_swing.
    :type subgait_id: str
    :param joint_names: Names of the joints
    :type joint_names: list
    :param position_x: x-coordinate of the desired foot location in meters.
    :type position_x: float
    :param position_y: y-coordinate of the desired foot location in meters. Default is zero.
    :type position_y: float
    """

    def __init__(
        self,
        gait_selection_node: Node,
        starting_position: dict,
        subgait_id: str,
        joint_names: List[str],
        position: Point,
        stop: bool,
    ):
        self._get_parameters(gait_selection_node)
        self.time = [
            0,
            self.push_off_fraction * self.duration,
            self.middle_point_fraction * self.duration,
            self.duration,
        ]
        self.starting_position = starting_position
        self.position = position
        self.joint_names = joint_names
        self.subgait_id = subgait_id
        self.stop = stop
        self.pose = Pose()

    def _get_extra_ankle_setpoint(self) -> Setpoint:
        """Returns an extra setpoint for the swing leg ankle
        that can be used to create a push off.

        :returns: An extra setpoint for the swing leg ankle
        :rtype: Setpoint
        """
        return Setpoint(Duration(self.time[1]), self.push_off_position, 0.0)

    def _solve_middle_setpoint(self) -> None:
        """Calls IK solver to compute the joint angles needed for the middle setpoint

        :returns: A setpoint_dict for the middle position.
        :rtype: dict
        """
        middle_position = self.pose.solve_mid_position(
            self.position.x,
            self.position.y,
            self.middle_point_fraction,
            self.middle_point_height,
            self.subgait_id,
        )

        self.middle_setpoint_dict = self._from_list_to_setpoint(
            self.joint_names,
            middle_position,
            None,
            self.time[2],
        )

    def _solve_desired_setpoint(self) -> None:
        """Calls IK solver to compute the joint angles needed for the
        desired x and y coordinate"""
        if self.stop:
            self.desired_position = self._from_joint_dict_to_list(
                get_position_from_yaml("stand")
            )
        else:
            self.desired_position = self.pose.solve_end_position(
                self.position.x, self.position.y, self.subgait_id
            )

        self.desired_setpoint_dict = self._from_list_to_setpoint(
            self.joint_names, self.desired_position, None, self.time[-1]
        )

    def _to_joint_trajectory_class(self) -> None:
        """Creates a list of DynamicJointTrajectories for each joint"""
        self.joint_trajectory_list = []
        for name in self.joint_names:
            setpoint_list = [
                self.starting_position[name],
                self.middle_setpoint_dict[name],
                self.desired_setpoint_dict[name],
            ]

            if (name == "right_ankle" and self.subgait_id == "right_swing") or (
                name == "left_ankle" and self.subgait_id == "left_swing"
            ):
                setpoint_list.insert(
                    EXTRA_ANKLE_SETPOINT_INDEX, self._get_extra_ankle_setpoint()
                )

            self.joint_trajectory_list.append(DynamicJointTrajectory(setpoint_list))

    def get_joint_trajectory_msg(self) -> trajectory_msg.JointTrajectory:
        """Return a joint_trajectory_msg containing the interpolated
        trajectories for each joint

        :returns: A joint_trajectory_msg
        :rtype: joint_trajectory_msg
        """
        # Update pose:
        pose_list = [joint.position for joint in self.starting_position.values()]
        self.pose = Pose(pose_list)

        self._solve_middle_setpoint()
        self._solve_desired_setpoint()
        self._get_extra_ankle_setpoint()

        # Create joint_trajectory_msg
        self._to_joint_trajectory_class()
        joint_trajectory_msg = trajectory_msg.JointTrajectory()
        joint_trajectory_msg.joint_names = self.joint_names

        for timestamp in self.time:
            joint_trajecory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajecory_point.time_from_start = Duration(timestamp).to_msg()

            for joint_trajectory in self.joint_trajectory_list:
                interpolated_setpoint = joint_trajectory.get_interpolated_setpoint(
                    timestamp
                )

                joint_trajecory_point.positions.append(interpolated_setpoint.position)
                joint_trajecory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajecory_point)

        return joint_trajectory_msg

    def get_final_position(self) -> dict:
        """Get setpoint_dictionary of the final setpoint.

        :return: The final setpoint of the subgait.
        :rtype: dict
        """
        return self._from_list_to_setpoint(
            self.joint_names, self.desired_position, None, self.time[0]
        )

    def _from_list_to_setpoint(
        self,
        joint_names: List[str],
        position: List[float],
        velocity: List[float],
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

        for i in range(len(joint_names)):
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

    def _from_joint_dict_to_list(self, joint_dict: dict) -> List[float]:
        return list(joint_dict.values())

    def _get_parameters(self, gait_selection_node: Node) -> None:
        """Gets the dynamic gait parameters from the gait_selection_node

        :param gait_selection_node: the gait selection node
        :type gait_selection_node: Node
        """
        self.duration = gait_selection_node.dynamic_subgait_duration
        self.middle_point_height = gait_selection_node.middle_point_height
        self.middle_point_fraction = gait_selection_node.middle_point_fraction
        self.push_off_fraction = gait_selection_node.push_off_fraction
        self.push_off_position = gait_selection_node.push_off_position

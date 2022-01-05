import numpy as np

from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import (
    DynamicJointTrajectory,
)
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from trajectory_msgs import msg as trajectory_msg
from march_goniometric_ik_solver.ik_solver import (
    solve_mid_position,
    solve_end_position,
    Pose,
)

# Middle point is always 0.1 meters higher than desired location
MIDDLE_POINT_HEIGHT = 0.1


class DynamicSubgait:
    """Creates joint trajectories based on the desired foot location.

    :param duration: Duration of the subgait.
    :type duration: float
    :param mid_point_fraction: Fraction of the subgait at which the middle setpoint will be set.
    :type mid_point_fraction: float
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
        duration,
        mid_point_fraction,
        starting_position,
        subgait_id,
        joint_names,
        position_x,
        position_y,
    ):
        self.mid_point_fraction = mid_point_fraction
        self.time = [0, self.mid_point_fraction * duration, duration]
        self.starting_position = starting_position
        self.position_x = position_x
        self.position_y = position_y
        self.joint_names = joint_names
        self.subgait_id = subgait_id

    def _middle_setpoint(self, position_x, position_y):
        """Calls IK solver to compute the joint angles needed for the middle setpoint

        :param position_x: x-coordinate in meters of the foot for the desired middle setpoint.
        :type position_x: float
        :param position_y: y-coordinate in meters of the foot for the desired middle setpoint.
        :type position_y: float

        :returns: A setpoint_dict for the middle position.
        :rtype: dict
        """
        middle_position = solve_mid_position(position_x, position_y, self.subgait_id)

        self.middle_setpoint_dict = self._from_list_to_setpoint(
            self.joint_names,
            middle_position,
            None,
            self.time[1],
        )

    def _desired_setpoint(self, position_x, position_y=0):
        """Calls IK solver to compute the joint angles needed for the desired x and y coordinate

        :param position_x: x-coordinate in meters of the desired foot location.
        :type position_x: float
        :param position_y: Optional y-coordinate in meters of the desired foot location. Default is zero.
        :type position_y: float
        """
        self.desired_position = solve_end_position(
            position_x, position_y, self.subgait_id
        )

        self.desired_setpoint_dict = self._from_list_to_setpoint(
            self.joint_names, self.desired_position, None, self.time[-1]
        )

    def _to_joint_trajectory_class(self):
        """Creates a list of DynamicJointTrajectories for each joint"""
        self.joint_trajectory_list = []
        for name in self.joint_names:
            self.joint_trajectory_list.append(
                DynamicJointTrajectory(
                    [
                        self.starting_position[name],
                        self.middle_setpoint_dict[name],
                        self.desired_setpoint_dict[name],
                    ]
                )
            )

    def get_joint_trajectory_msg(self) -> trajectory_msg.JointTrajectory:
        """Return a joint_trajectory_msg containing the interpolated trajectories for each joint

        :returns: A joint_trajectory_msg
        :rtype: joint_trajectory_msg
        """
        # Solve for middle setpoint.
        starting_position_list = []
        for joint in self.joint_names:
            starting_position_list.append(self.starting_position[joint].position)

        # Swing leg ankle position is relative to stance leg ankle position (0,0)
        current_pose = Pose(starting_position_list)
        stance_swing_dis = current_pose.get_ankle_distance()

        # Middle position x is weighted average of the current and desired setpoint,
        # relative to the stance leg ankle
        mid_pos_x = self.mid_point_fraction * self.position_x - stance_swing_dis

        # Middle position y should always be heigher than the desired position y
        mid_pos_y = self.position_y + MIDDLE_POINT_HEIGHT

        self._middle_setpoint(mid_pos_x, mid_pos_y)

        # Solve for desired setpoint
        self._desired_setpoint(self.position_x, position_y=self.position_y)

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

    def _get_final_position(self):
        """Get setpoint_dictionary of the final setpoint.

        :return: The final setpoint of the subgait.
        :rtype: dict
        """
        return self._from_list_to_setpoint(
            self.joint_names, self.desired_position, None, self.time[0]
        )

    def _from_list_to_setpoint(self, joint_names, position, velocity, time):
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

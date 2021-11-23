from dynamic_joint_trajectory_setpoint import DynamicJointTrajectorySetpoint
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from trajectory_msgs import msg as trajectory_msg
import numpy as np


class DynamicSubgait:
    """class that reads setpoints and returns list of jointtrajectories"""

    def __init__(
        self, time, current_state, middle_position, desired_position, desired_velocity
    ):
        self.joints = [
            "left_ankle",
            "left_knee",
            "left_hip_fe",
            "left_hip_aa",
            "right_hip_aa",
            "right_hip_fe",
            "right_knee",
            "right_ankle",
        ]
        self.time = time
        self.current_state = current_state
        self.middle_state = middle_position
        self.desired_position = desired_position
        self.desired_velocity = desired_velocity

    def current_setpoint(self):
        """Reads current state of the robot"""
        self.current_setpoint_dict = self.from_list_to_setpoint(
            self.current_state.joint_names,
            self.current_state.actual.positions,
            self.current_state.actual.velocities,
            self.time[0],
        )

    def middle_setpoint(self):
        """Returns the middle setpoint. Fixed for now.
        Should adapt dynamically in the future"""
        self.middle_setpoint_dict = self.from_list_to_setpoint(
            self.joints,
            self.middle_state,
            None,
            self.time[1],
        )

    def desired_setpoint(self):
        """Calls IK solver to compute setpoint from CoViD location"""
        self.desired_setpoint_dict = self.from_list_to_setpoint(
            self.joints, self.desired_position, self.desired_velocity, self.time[2]
        )

    def to_joint_trajectory_class(self):
        """Returns a list of JointTrajectory classes containing
        the setpoints for each joint"""
        self.joint_trajectory_list = []
        for name in self.joints:
            self.joint_trajectory_list.append(
                DynamicJointTrajectorySetpoint(
                    [
                        self.current_setpoint_dict[name],
                        self.middle_setpoint_dict[name],
                        self.desired_setpoint_dict[name],
                    ]
                )
            )

    def to_joint_trajectory_msg(self):
        """Returns a joint_trajectory_msg which can be send to the exo"""
        self.current_setpoint()
        self.middle_setpoint()
        self.desired_setpoint()
        self.to_joint_trajectory_class()

        joint_trajectory_msg = trajectory_msg.JointTrajectory()
        joint_trajectory_msg.joint_names = self.joints

        timestamps = np.linspace(self.time[0], self.time[-1], 20)

        for timestamp in timestamps:
            joint_trajecory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajecory_point.time_from_start = Duration(timestamp).to_msg()

            for joint_trajectory in self.joint_trajectory_list:
                joint_trajectory.interpolate_setpoints()
                interpolated_setpoint = joint_trajectory.get_interpolated_setpoint(
                    timestamp
                )

                joint_trajecory_point.positions.append(interpolated_setpoint.position)
                joint_trajecory_point.velocities.append(interpolated_setpoint.velocity)

            joint_trajectory_msg.points.append(joint_trajecory_point)

        return joint_trajectory_msg

    def from_list_to_setpoint(self, joint_names, position, velocity, time):
        """Computes setpoint dictionary from JointState msg"""
        setpoint_dict = {}

        if velocity is not None:
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
        else:
            for i in range(len(joint_names)):
                setpoint_dict.update(
                    {
                        joint_names[i]: Setpoint(
                            Duration(time),
                            position[i],
                            0.0,
                        )
                    }
                )

        return setpoint_dict

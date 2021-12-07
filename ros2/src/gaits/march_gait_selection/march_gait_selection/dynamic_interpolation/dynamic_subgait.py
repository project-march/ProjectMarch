from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory_setpoint import DynamicJointTrajectorySetpoint
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from trajectory_msgs import msg as trajectory_msg
import numpy as np
from march_goniometric_ik_solver.ik_solver import solve_ik


class DynamicSubgait:
    """class that reads setpoints and returns list of jointtrajectories"""

    def __init__(self, time, starting_position, swing_leg, position_x, position_y=0):
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
        self.starting_position = starting_position
        self.position_x = position_x
        self.position_y = position_y
        self.swing_leg = swing_leg

    def middle_setpoint(self):
        """Returns the middle setpoint. Fixed for now.
        Should adapt dynamically in the future"""
        self.middle_position = [0.0, 0.14, 0.1, 0.03, 0.03, 0.61, 1.17, 0.0]
        if self.swing_leg == "left":
            self.middle_position.reverse()

        self.middle_setpoint_dict = self.from_list_to_setpoint(
            self.joints,
            self.middle_position,
            None,
            self.time[1],
        )

    def desired_setpoint(self, position_x, position_y=0):
        """Calls IK solver to compute setpoint from CoViD location.
        Position is defined in centimeters and takes two argurments:
        forward distance and height. Ankle RoM should be given in degrees"""
        self.desired_position = solve_ik(position_x, position_y)
        if self.swing_leg == "left":
            self.desired_position.reverse()

        if self.desired_position[0] > 0.1745 or self.desired_position[-1] > 0.1745:
            dorsiflexion = (
                max([self.desired_position[0], self.desired_position[-1]]) / 3.14 * 180
            )
            print(
                f"Dorsiflexion bigger than 10 degrees: {round(dorsiflexion, 1)} degrees",
            )

        self.desired_setpoint_dict = self.from_list_to_setpoint(
            self.joints, self.desired_position, None, self.time[2]
        )

    def get_final_position(self):
        # SHOULD FIX FIXED TIME DELAY OF 0.2 SEC
        return self.from_list_to_setpoint(
            self.joints, self.desired_position, None, self.time[0]
        )

    def to_joint_trajectory_class(self):
        """Returns a list of JointTrajectory classes containing
        the setpoints for each joint"""

        self.joint_trajectory_list = []
        for name in self.joints:
            self.joint_trajectory_list.append(
                DynamicJointTrajectorySetpoint(
                    [
                        self.starting_position[name],
                        self.middle_setpoint_dict[name],
                        self.desired_setpoint_dict[name],
                    ]
                )
            )

    def to_joint_trajectory_msg(self):
        """Returns a joint_trajectory_msg which can be send to the exo"""
        self.middle_setpoint()
        self.desired_setpoint(self.position_x, position_y=self.position_y)
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
        """Computes setpoint dictionary from a list"""
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

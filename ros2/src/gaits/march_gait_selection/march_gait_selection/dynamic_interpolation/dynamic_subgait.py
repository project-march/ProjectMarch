import numpy as np

from dynamic_joint_trajectory_setpoint import DynamicJointTrajectorySetpoint
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.duration import Duration
from trajectory_msgs import msg as trajectory_msg
from march_goniometric_ik_solver.ik_solver import (
    solve_mid_position,
    solve_end_position,
    calculate_joint_positions,
)


class DynamicSubgait:
    """class that reads setpoints and returns list of jointtrajectories"""

    def __init__(
        self,
        duration,
        mid_point_frac,
        current_state,
        swing_leg,
        position_x,
        position_y=0,
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
        self.delay = 0.2
        self.time = [
            self.delay,
            self.delay + mid_point_frac * duration,
            self.delay + duration,
        ]
        self.mid_point_frac = mid_point_frac
        self.current_state = current_state
        self.position_x = position_x
        self.position_y = position_y
        self.swing_leg = swing_leg

    def current_setpoint(self):
        """Reads current state of the robot"""
        # Current state velocity readings not correct. Sometimes reported high
        # velocities in the ankle joints, which leads to invalid gaits
        self.current_setpoint_dict = self.from_list_to_setpoint(
            self.current_state.joint_names,
            self.current_state.actual.positions,
            None,
            self.time[0],
        )

    def middle_setpoint(self, position_x, position_y):
        """Returns the middle setpoint. Fixed for now.
        Should adapt dynamically in the future"""
        middle_position = solve_mid_position(position_x, position_y)
        if self.swing_leg == "left":
            middle_position.reverse()

        self.middle_setpoint_dict = self.from_list_to_setpoint(
            self.joints,
            middle_position,
            None,
            self.time[1],
        )

    def desired_setpoint(self, position_x, position_y=0):
        """Calls IK solver to compute setpoint from CoViD location.
        Position is defined in centimeters and takes two argurments:
        forward distance and height. Ankle RoM should be given in degrees"""
        desired_position = solve_end_position(position_x, position_y)
        if self.swing_leg == "left":
            desired_position.reverse()

        if desired_position[0] > 0.1745 or desired_position[-1] > 0.1745:
            dorsiflexion = max([desired_position[0], desired_position[-1]]) / 3.14 * 180
            print(
                f"Dorsiflexion bigger than 10 degrees: {round(dorsiflexion, 1)} degrees",
            )

        self.desired_setpoint_dict = self.from_list_to_setpoint(
            self.joints, desired_position, None, self.time[2]
        )

    def to_joint_trajectory_class(self):
        """Returns a list of JointTrajectory classes containing
        the setpoints for each joint"""
        # fix hardcoded haa/false current state readings
        self.current_setpoint_dict.update(
            {
                "right_hip_aa": Setpoint(
                    Duration(0.2),
                    0.03,
                    0.0,
                )
            }
        )

        self.current_setpoint_dict.update(
            {
                "left_hip_aa": Setpoint(
                    Duration(0.2),
                    0.03,
                    0.0,
                )
            }
        )

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
        # Read current setpoint
        self.current_setpoint()

        # Solve for middle setpoint. solve_ik function cannot handle HAA yet
        current_state_without_haa = []
        for joint in self.joints:
            current_state_without_haa.append(
                np.rad2deg(self.current_setpoint_dict[joint].position)
            )

        del current_state_without_haa[4]
        del current_state_without_haa[3]

        if self.swing_leg == "left":
            current_state_without_haa.reverse()

        swing_leg_ankle_pos = calculate_joint_positions(
            current_state_without_haa, joint="pos_ankle2"
        )

        # Swing leg ankle position is relative to stance leg ankle position (0,0)
        stance_swing_dis = np.sqrt(
            swing_leg_ankle_pos[0] ** 2 + swing_leg_ankle_pos[1] ** 2
        )

        # Middle position x is weighted average of the current and desired setpoint,
        # relative to the stance leg ankle
        mid_pos_x = self.mid_point_frac * self.position_x - stance_swing_dis

        # Middle position y should always be heigher than the desired position y
        mid_pos_y = self.position_y + 10

        self.middle_setpoint(mid_pos_x, mid_pos_y)

        # Solve for desired setpoint
        self.desired_setpoint(self.position_x, position_y=self.position_y)
        self.to_joint_trajectory_class()

        # Create joint_trajectory_msg
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

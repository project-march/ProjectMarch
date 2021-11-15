import rospy
from trajectory_msgs import msg as trajectory_msg
from scipy.interpolate import CubicSpline
import numpy as np

# Needed to get current state
from sensor_msgs.msg import JointState


class DynamicJointTrajectory:
    """Testing class for interpolating between three dynamic setpoints.
    Based on the JointTrajectoryClass"""

    def __init__(self):
        self.time = [0, 0.5, 1]

    def get_current_state(self, msg: JointState):
        """Reads current joint angles and velocities of the exo"""
        self._joint_names = msg.name
        self._current_position = msg.position
        self._current_velocity = msg.velocity

    def set_middle_state(self):
        """Set middle state (currently fixed)"""
        self._middle_position = np.deg2rad([0.0, 8.0, -2.0, 2.0, 2.0, 35.0, 67.0, 0.0])

    def get_desired_state(self):
        """Reads desired foot location as given by CoViD and performs
        inverse kinematics to return desired joint angles"""

        # Dummy position until covid has a topic. Maybe publish/make a node
        # that publishes fake feasible states
        self._desired_position = np.deg2rad([0.0, 8.0, -9.5, 2.0, 2.0, 18.0, 8.0, 0.0])
        self._desired_velocity = np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, -20, 0.0, 0.0])

    def interpolate_setpoints(self):
        """Interpolates between the current, middle and desired state
        with a cubic interpolation. Velocity boundary conditions are set
        at the start and end state of the subgait."""
        position = [
            self._current_position,
            self._middle_position,
            self._desired_position,
        ]
        velocity = [self._current_velocity, self._desired_velocity]
        boundary_condition = ((1, velocity[0]), (1, velocity[-1]))

        yi = []
        for i in range(len(position)):
            yi.append(position[i])

        self.interpolated_position = CubicSpline(
            self.time, yi, bc_type=boundary_condition
        )
        self.interpolated_velocity = self.interpolated_position.derivative()

        return (
            self.interpolated_position,
            self.interpolated_velocity,
        )

    def to_joint_trajectory_msg(self):
        """Create trajectory msg of the interpolated setpoints for the publisher.
        See same function in subgait.py"""

        joint_trajectory_msg = trajectory_msg.JointTrajectory()

        joint_trajectory_msg.joint_names = self._joint_names

        for timestamp in self.time:
            joint_trajectory_point = trajectory_msg.JointTrajectoryPoint()
            joint_trajectory_point.time_from_start = rospy.Duration.from_sec(timestamp)

            for i in range(len(self._joint_names)):
                joint_trajectory_point.positions.append(
                    self.interpolated_position(timestamp)[i]
                )
                joint_trajectory_point.velocities.append(
                    self.interpolated_velocity(timestamp)[i]
                )

            joint_trajectory_msg.points.append(joint_trajectory_point)

        return joint_trajectory_msg

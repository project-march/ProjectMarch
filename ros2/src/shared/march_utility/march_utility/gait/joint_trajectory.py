"""This module contain the JointTrajectory class.

This is used for creating the positions that should be followed by the joints.
"""

from __future__ import annotations

from math import isclose
from typing import List, Tuple, Any, Optional

import rclpy

from march_utility.exceptions.gait_exceptions import (
    SubgaitInterpolationError,
    NonValidGaitContentError,
)
from march_utility.utilities.duration import Duration
from scipy.interpolate import BPoly
from urdf_parser_py import urdf

from .setpoint import Setpoint

ALLOWED_ERROR = 0.001


class JointTrajectory:
    """Base class for joint trajectory of a gait."""

    setpoint_class = Setpoint

    def __init__(self, name: str, setpoints: List[Setpoint], duration: Duration) -> None:
        """Initialize a joint trajectory."""
        self.name = name
        self._setpoints = setpoints
        self._duration = round(duration, self.setpoint_class.digits)
        self.interpolated_position: Any = None
        self.interpolated_velocity: Any = None
        self.interpolate_setpoints()

    @classmethod
    def from_setpoint_dict(cls, name: str, setpoint_dict: List[dict], duration: Duration) -> JointTrajectory:
        """Creates a list of joint trajectories.

        Args:
            name (str): Name of the joint.
            setpoint_dict (List[dict]): A list of setpoints from the subgait configuration.
            duration (Duration): The total duration of the trajectory.
        """
        setpoints = [
            Setpoint(
                time=Duration(nanoseconds=setpoint["time_from_start"]),
                position=setpoint["position"],
                velocity=setpoint["velocity"],
            )
            for setpoint in setpoint_dict
        ]
        return cls(name, setpoints, duration)

    @staticmethod
    def get_joint_from_urdf(robot: urdf.Robot, joint_name: str) -> Optional[urdf.Joint]:
        """Get the joint from the urdf robot with the given joint name.

        Args:
            robot (urdf.Robot): The urdf robot to use.
            joint_name (str): The name to look for.
        """
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

    @property
    def duration(self) -> Duration:
        """Duration: Get the duration of the joint trajectory."""
        return self._duration

    def set_duration(self, new_duration: Duration, rescale: bool = True) -> None:
        """Change the duration of the joint trajectory.

        Args:
            new_duration (Duration): The new duration to change to.
            rescale (bool): If `True`, the trajectory will be shortened/lengthed by rescaling.
                If `False`, the setpoints outside of the duration will be removed.
        """
        for setpoint in reversed(self.setpoints):
            if rescale:
                setpoint.time = setpoint.time * (new_duration / self.duration)
                setpoint.velocity = setpoint.velocity * (self.duration / new_duration)
            else:
                if setpoint.time > new_duration:
                    self.setpoints.remove(setpoint)

        self._duration = round(new_duration, self.setpoint_class.digits)
        self.interpolate_setpoints()

    def from_begin_point(self, begin_time: Duration, position: float) -> None:
        """Manipulate the gait to start at given time.

        Removes all set points after the given begin time. Adds the begin position
        with 0 velocity at the start. It also adds 1 second at the beginning
        to allow speeding up to the required speed again.

        Args:
            begin_time (Duration): The time to start.
            position (float): The position to start from.
        """
        begin_time -= Duration(seconds=1)
        for setpoint in reversed(self.setpoints):
            if setpoint.time <= begin_time:
                self.setpoints.remove(setpoint)
            else:
                setpoint.time -= begin_time
        self.setpoints = [Setpoint(time=Duration(), position=position, velocity=0)] + self.setpoints
        self._duration = self._duration - begin_time

    @property
    def setpoints(self) -> List[Setpoint]:
        """List[Setpoint]. All the setpoints in the trajectory."""
        return self._setpoints

    @setpoints.setter
    def setpoints(self, setpoints: List[Setpoint]):
        self._setpoints = setpoints
        self.interpolate_setpoints()

    def get_setpoints_unzipped(self) -> Tuple[List[float], List[float], List[float]]:
        """Get all the listed attributes of the setpoints.

        Converts the setpoints time to seconds.

        Returns::
            Tuple(
                List[float]: List of the time in seconds for every setpoint,
                List[float]: List of the positions (angles) for the joint for every setpoint,
                List[float]: List of the velocities (angles) for the joint for every setpoint
            )
        """
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time.seconds)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def validate_joint_transition(self, joint: JointTrajectory) -> bool:
        """Validate the ending and starting of this joint to a given joint.

        Args:
            joint (JointTrajectory): The joint of the next subgait (not the previous one).

        Returns:
            bool. `True` if ending and starting point are identical else `False`.
        """
        if not self._validate_boundary_points():
            raise NonValidGaitContentError(
                self.name,
                msg=f"Invalid boundary points for begin setpoint {self.setpoints[0]} "
                f"and end setpoint {self.setpoints[-1]} with duration {self.duration}",
            )

        from_setpoint = self.setpoints[-1]
        to_setpoint = joint.setpoints[0]

        logger = rclpy.logging.get_logger("march_utility_logger")

        if abs(from_setpoint.velocity - to_setpoint.velocity) > ALLOWED_ERROR:
            logger.warning(
                f"joint {self.name} has an invalid velocity transition as {from_setpoint.velocity} != {to_setpoint.velocity}"
            )
            return False

        if abs(from_setpoint.position - to_setpoint.position) > ALLOWED_ERROR:
            logger.warning(
                f"joint {self.name} has an invalid position transition as {from_setpoint.position} != {to_setpoint.position}"
            )
            return False

        return True

    def _validate_boundary_points(self) -> bool:
        """Validate the starting and ending of this joint.

        Returns: `False` if the starting/ending point is (not at 0/duration) and (has nonzero speed), `True` otherwise.
        """
        return (self.setpoints[0].time.nanoseconds == 0 or self.setpoints[0].velocity == 0) and (
            isclose(
                self.setpoints[-1].time.seconds,
                self.duration.seconds,
                abs_tol=ALLOWED_ERROR,
            )
            or self.setpoints[-1].velocity == 0
        )

    def interpolate_setpoints(self) -> None:
        """Interpolate the setpoints from the joint trajectory."""
        if len(self.setpoints) == 1:
            self.interpolated_position = lambda time: self.setpoints[0].position
            self.interpolated_velocity = lambda time: self.setpoints[0].velocity
            return

        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(len(time)):
            yi.append([position[i], velocity[i]])

        # We do a cubic spline here, like the ros joint_trajectory_action_controller,
        # https://wiki.ros.org/robot_mechanism_controllers/JointTrajectoryActionController
        self.interpolated_position = BPoly.from_derivatives(time, yi)
        self.interpolated_velocity = self.interpolated_position.derivative()

    def get_interpolated_setpoint(self, time: Duration) -> Setpoint:
        """Get a setpoint on a certain time.

        If there is no setpoint at this time point, it will interpolate
        between the setpoints.
        """
        if time < Duration(seconds=0):
            return self.setpoint_class(time, self.setpoints[0].position, 0)
        if time > self.duration:
            return self.setpoint_class(time, self.setpoints[-1].position, 0)

        return self.setpoint_class(
            time,
            float(self.interpolated_position(time.seconds)),
            float(self.interpolated_velocity(time.seconds)),
        )

    def __getitem__(self, index) -> Setpoint:
        """Setpoint. Returns the setpoint at the given index."""
        return self.setpoints[index]

    def __len__(self) -> int:
        """int. Returns the amount of setpoints."""
        return len(self.setpoints)

    @staticmethod
    def interpolate_joint_trajectories(
        base_trajectory: JointTrajectory,
        other_trajectory: JointTrajectory,
        parameter: float,
    ) -> JointTrajectory:
        """Linearly interpolate two joint trajectories with the parameter.

        Args:
            base_trajectory (JointTrajectory): The base trajectory to interpolate from.
            other_trajectory (JointTrajectory): The other trajectory to interpolate to.
            parameter (float): The parameter to use for interpolation. Should be 0 <= parameter <= 1

        Returns:
            JointTrajectory. The interpolated trajectory:
                * `base_trajectory`, if `parameter` == 0.
                * `other_trajectory`, if `parameter` == 0.
        """
        if len(base_trajectory.setpoints) != len(other_trajectory.setpoints):
            raise SubgaitInterpolationError(
                "The amount of setpoints do not match for joint {0}".format(base_trajectory.name)
            )
        setpoints = []

        for base_setpoint, other_setpoint in zip(base_trajectory.setpoints, other_trajectory.setpoints):
            interpolated_setpoint_to_add = JointTrajectory.setpoint_class.interpolate_setpoints(
                base_setpoint, other_setpoint, parameter
            )
            setpoints.append(interpolated_setpoint_to_add)

        duration = base_trajectory.duration.weighted_average(other_trajectory.duration, parameter)
        return JointTrajectory(base_trajectory.name, setpoints, duration)

    @staticmethod
    def check_joint_interpolation_is_safe(
        base_joint: JointTrajectory,
        other_joint: JointTrajectory,
    ) -> bool:
        """Check whether it is possible to interpolate between the two joint trajectories.

        Args:
            base_joint: The first joint trajectory to interpolate.
            other_joint: The second joint trajectory to interpolate.
        """
        if base_joint.name != other_joint.name:
            raise SubgaitInterpolationError(
                f"The subgaits to interpolate do not have the same joints, base"
                f" subgait has {base_joint.name}, while other subgait has "
                f"{other_joint.name}"
            )
            # check whether each base joint as its corresponding other joint
        return True

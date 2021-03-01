"""
This module contain the JointTrajectory class.

This is used for for creating the positions that should be followed by the joints,
and to check the safety limits.
"""

from __future__ import annotations

from typing import List, Tuple, Any

from march_utility.exceptions.gait_exceptions import (
    SubgaitInterpolationError,
    NonValidGaitContent,
)
from march_utility.utilities.duration import Duration
from scipy.interpolate import BPoly
from urdf_parser_py import urdf

from .limits import Limits
from .setpoint import Setpoint

ALLOWED_ERROR = 0.001


class JointTrajectory(object):
    """Base class for joint trajectory of a gait."""

    setpoint_class = Setpoint

    def __init__(
        self,
        name: str,
        limits: Limits,
        setpoints: List[Setpoint],
        duration: Duration,
    ) -> None:
        """Initialize a joint trajectory."""
        self.name = name
        self.limits = limits
        self._setpoints = setpoints
        self._duration = round(duration, self.setpoint_class.digits)
        self.interpolated_position: Any = None
        self.interpolated_velocity: Any = None
        self.interpolate_setpoints()

    @classmethod
    def from_setpoint_dict(
        cls, name: str, limits: Limits, setpoint_dict: List[dict], duration: Duration
    ) -> JointTrajectory:
        """Creates a list of joint trajectories.

        :param str name: Name of the joint
        :param limits: Joint limits from the URDF
        :param list(dict) setpoints: A list of setpoints from the subgait configuration
        :param duration: The total duration of the trajectory
        """
        setpoints = [
            Setpoint(
                time=Duration(nanoseconds=setpoint["time_from_start"]),
                position=setpoint["position"],
                velocity=setpoint["velocity"],
            )
            for setpoint in setpoint_dict
        ]
        return cls(name, limits, setpoints, duration)

    @staticmethod
    def get_joint_from_urdf(robot: urdf.Robot, joint_name: str) -> urdf.Joint:
        """Get the joint from the urdf robot with the given joint name.

        :param robot: The urdf robot to use.
        :param joint_name: The name to look for.
        """
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

    @property
    def duration(self) -> Duration:
        """Get the duration of the joint trajectory."""
        return self._duration

    def set_duration(self, new_duration: Duration, rescale: bool = True) -> None:
        """Change the duration of the joint trajectory.

        :param new_duration: The new duration to change to.
        :param rescale: If true, the trajectory will be shortened/lengthed by rescaling.
            Else, the setpoints outside of the duration will be removed.
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
        """
        Manipulate the gait to start at given time.

        Removes all set points after the given begin time. Adds the begin position
        with 0 velocity at the start. It also adds 1 second at the beginning
        to allow speeding up to the required speed again.
        :param begin_time: The time to start
        :param position: The position to start from
        """
        begin_time -= Duration(seconds=1)
        for setpoint in reversed(self.setpoints):
            if setpoint.time <= begin_time:
                self.setpoints.remove(setpoint)
            else:
                setpoint.time -= begin_time
        self.setpoints = [
            Setpoint(time=Duration(), position=position, velocity=0)
        ] + self.setpoints
        self._duration = self._duration - begin_time

    @property
    def setpoints(self) -> List[Setpoint]:
        return self._setpoints

    @setpoints.setter
    def setpoints(self, setpoints: List[Setpoint]):
        self._setpoints = setpoints
        self.interpolate_setpoints()

    def get_setpoints_unzipped(self) -> Tuple[List[float], List[float], List[float]]:
        """Get all the listed attributes of the setpoints.

        Converts the setpoints time to seconds.
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

        :param joint:
            the joint of the next subgait (not the previous one)

        :returns:
            True if ending and starting point are identical else False
        """
        if not self._validate_boundary_points():
            raise NonValidGaitContent(
                self.name,
                msg=f"Invalid boundary points for begin setpoint {self.setpoints[0]} "
                f"and end setpoint {self.setpoints[-1]}",
            )

        from_setpoint = self.setpoints[-1]
        to_setpoint = joint.setpoints[0]

        return (
            abs(from_setpoint.velocity - to_setpoint.velocity) <= ALLOWED_ERROR
            and abs(from_setpoint.position - to_setpoint.position) <= ALLOWED_ERROR
        )

    def _validate_boundary_points(self) -> bool:
        """Validate the starting and ending of this joint.

        :returns:
            False if the starting/ending point is (not at 0/duration) and
            (has nonzero speed), True otherwise
        """
        return (
            self.setpoints[0].time.nanoseconds == 0 or self.setpoints[0].velocity == 0
        ) and (
            self.setpoints[-1].time == round(self.duration, Setpoint.digits)
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
        for i in range(0, len(time)):
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

    def __getitem__(self, index):
        return self.setpoints[index]

    def __len__(self):
        return len(self.setpoints)

    @staticmethod
    def interpolate_joint_trajectories(
        base_trajectory: JointTrajectory,
        other_trajectory: JointTrajectory,
        parameter: float,
    ) -> JointTrajectory:
        """Linearly interpolate two joint trajectories with the parameter.

        :param base_trajectory:
            base trajectory, return value if parameter is equal to zero
        :param other_trajectory:
            other trajectory, return value if parameter is equal to one
        :param parameter:
            The parameter to use for interpolation. Should be 0 <= parameter <= 1

        :return:
            The interpolated trajectory
        """
        if base_trajectory.limits != other_trajectory.limits:
            raise SubgaitInterpolationError(
                f"Not able to safely interpolate because limits "
                f"are not equal for joints {base_trajectory.name} "
                f"and {other_trajectory.name}"
            )

        if len(base_trajectory.setpoints) != len(other_trajectory.setpoints):
            raise SubgaitInterpolationError(
                "The amount of setpoints do not match for joint {0}".format(
                    base_trajectory.name
                )
            )
        setpoints = []

        for base_setpoint, other_setpoint in zip(
            base_trajectory.setpoints, other_trajectory.setpoints
        ):
            interpolated_setpoint_to_add = (
                JointTrajectory.setpoint_class.interpolate_setpoints(
                    base_setpoint, other_setpoint, parameter
                )
            )
            setpoints.append(interpolated_setpoint_to_add)

        duration = base_trajectory.duration.weighted_average(
            other_trajectory.duration, parameter
        )
        return JointTrajectory(
            base_trajectory.name, base_trajectory.limits, setpoints, duration
        )

    @staticmethod
    def check_joint_interpolation_is_safe(
        base_joint: JointTrajectory,
        other_joint: JointTrajectory,
    ) -> bool:
        """Check whether it is possible to interpolate between the two joint trajectories.

        :param base_joint: The first joint trajectory to interpolate.
        :param other_joint: The second joint trajectory to interpolate.
        """
        if base_joint.name != other_joint.name:
            raise SubgaitInterpolationError(
                f"The subgaits to interpolate do not have the same joints, base"
                f" subgait has {base_joint.name}, while other subgait has "
                f"{other_joint.name}"
            )
            # check whether each base joint as its corresponding other joint
        if base_joint.limits != other_joint.limits:
            raise SubgaitInterpolationError(
                f"Not able to safely interpolate because limits are not equal for "
                f"joints {base_joint.name} and {other_joint.name}"
            )
        return True

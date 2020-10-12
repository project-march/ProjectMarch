from typing import List

from rclpy.duration import Duration
from scipy.interpolate import BPoly
from urdf_parser_py import urdf

from .limits import Limits
from src.exceptions.gait_exceptions import SubgaitInterpolationError, NonValidGaitContent
from .setpoint import Setpoint


class JointTrajectory(object):
    """Base class for joint trajectory of a gait."""

    setpoint_class = Setpoint

    def __init__(self, name: str, limits: Limits, setpoints: List[Setpoint], duration: float, *args):
        self.name = name
        self.limits = limits
        self._setpoints = setpoints
        self._duration = round(duration, self.setpoint_class.digits)  # seconds
        self.interpolated_position = None
        self.interpolated_velocity = None
        self.interpolate_setpoints()

    @classmethod
    def from_setpoints(cls, name: str, limits: Limits, setpoints: List[dict], duration: float, *args):
        """Creates a list of joint trajectories.

        :param str name: Name of the joint
        :param limits: Joint limits from the URDF
        :param list(dict) setpoints: A list of setpoints from the subgait configuration
        :param duration: The total duration of the trajectory
        """
        setpoints = [
            cls.setpoint_class(
                time=Duration(seconds=setpoint['time_from_start']['secs'],
                              nanoseconds=setpoint['time_from_start']['nsecs']).nanoseconds * 1e-9,
                position=setpoint['position'], velocity=setpoint['velocity']) for setpoint in setpoints]
        return cls(name, limits, setpoints, duration, *args)

    @staticmethod
    def get_joint_from_urdf(robot: urdf.Robot, joint_name: str):
        """Get the name of the robot joint corresponding with the joint in the subgait."""
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

    @property
    def duration(self) -> float:
        return self._duration

    def set_duration(self, new_duration: float, rescale: bool = True) -> None:
        for setpoint in reversed(self.setpoints):
            if rescale:
                setpoint.time = setpoint.time * new_duration / self.duration
                setpoint.velocity = setpoint.velocity * self.duration / new_duration
            else:
                if setpoint.time > new_duration:
                    self.setpoints.remove(setpoint)

        self._duration = round(new_duration, self.setpoint_class.digits)
        self.interpolate_setpoints()

    @property
    def setpoints(self) -> List[Setpoint]:
        return self._setpoints

    @setpoints.setter
    def setpoints(self, setpoints: List[Setpoint]):
        self._setpoints = setpoints
        self.interpolate_setpoints()

    def get_setpoints_unzipped(self) -> (List[float], List[float], List[float]):
        """Get all the listed attributes of the setpoints."""
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def validate_joint_transition(self, joint) -> bool:
        """Validate the ending and starting of this joint to a given joint.

        :param joint:
            the joint of the next subgait (not the previous one)

        :returns:
            True if ending and starting point are identical else False
        """
        if not self._validate_boundary_points():
            raise NonValidGaitContent(self.name, msg=f'Invalid boundary points for begin setpoint {self.setpoints[0]} '
                                                     f'and end setpoint {self.setpoints[-1]}')

        from_setpoint = self.setpoints[-1]
        to_setpoint = joint.setpoints[0]
        print(f'validating velocity {from_setpoint.velocity} to {to_setpoint.velocity}')
        print(f'validating position {from_setpoint.position} to {to_setpoint.position}')
        if from_setpoint.velocity == to_setpoint.velocity and from_setpoint.position == to_setpoint.position:
            return True

        return False

    def _validate_boundary_points(self) -> bool:
        """Validate the starting and ending of this joint are at t = 0 and t = duration, or that their speed is zero.

        :returns:
            False if the starting/ending point is (not at 0/duration) and (has nonzero speed), True otherwise
        """
        return (self.setpoints[0].time == 0 or self.setpoints[0].velocity == 0) and \
               (self.setpoints[-1].time == round(self.duration, Setpoint.digits) or self.setpoints[-1].velocity == 0)

    def interpolate_setpoints(self) -> None:
        if len(self.setpoints) == 1:
            self.interpolated_position = lambda time: self.setpoints[0].position
            self.interpolated_velocity = lambda time: self.setpoints[0].velocity
            return

        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(0, len(time)):
            yi.append([position[i], velocity[i]])

        # We do a cubic spline here, just like the ros joint_trajectory_action_controller,
        # see https://wiki.ros.org/robot_mechanism_controllers/JointTrajectoryActionController
        self.interpolated_position = BPoly.from_derivatives(time, yi)
        self.interpolated_velocity = self.interpolated_position.derivative()

    def get_interpolated_setpoint(self, time) -> Setpoint:
        if time < 0:
            return self.setpoint_class(time, self.setpoints[0].position, 0)
        if time > self.duration:
            return self.setpoint_class(time, self.setpoints[-1].position, 0)

        return self.setpoint_class(time, float(self.interpolated_position(time)), float(self.interpolated_velocity(time)))

    def __getitem__(self, index):
        return self.setpoints[index]

    def __len__(self):
        return len(self.setpoints)

    @classmethod
    def interpolate_joint_trajectories(cls, base_trajectory, other_trajectory, parameter: float):
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
            raise SubgaitInterpolationError('Not able to safely interpolate because limits are not equal for joint {0}'.
                                            format(base_trajectory.name))
        if len(base_trajectory.setpoints) != len(other_trajectory.setpoints):
            raise SubgaitInterpolationError('The amount of setpoints do not match for joint {0}'.
                                            format(base_trajectory.name))
        setpoints = []
        for base_setpoint, other_setpoint in zip(base_trajectory.setpoints, other_trajectory.setpoints):
            setpoints.append(cls.setpoint_class.interpolate_setpoints(base_setpoint, other_setpoint, parameter))
        duration = parameter * base_trajectory.duration + (1 - parameter) * other_trajectory.duration
        return JointTrajectory(base_trajectory.name, base_trajectory.limits, setpoints, duration)

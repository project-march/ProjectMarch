import rospy
from scipy.interpolate import BPoly

from march_shared_classes.exceptions.gait_exceptions import SubgaitInterpolationError

from .setpoint import Setpoint

ALLOWED_ERROR = 0.001


class JointTrajectory(object):
    """Base class for joint trajectory of a gait."""

    setpoint_class = Setpoint

    def __init__(self, name, limits, setpoints, duration, *args):
        self.name = name
        self.limits = limits
        self._setpoints = setpoints
        self._duration = round(duration, self.setpoint_class.digits)
        self.interpolated_position = None
        self.interpolated_velocity = None
        self.interpolate_setpoints()

    @classmethod
    def from_setpoints(cls, name, limits, setpoints, duration, *args):
        """Creates a list of joint trajectories.

        :param str name: Name of the joint
        :param limits: Joint limits from the URDF
        :param list(dict) setpoints: A list of setpoints from the subgait configuration
        :param duration: The total duration of the trajectory
        """
        setpoints = [
            cls.setpoint_class(
                rospy.Duration(
                    setpoint["time_from_start"]["secs"],
                    setpoint["time_from_start"]["nsecs"],
                ).to_sec(),
                setpoint["position"],
                setpoint["velocity"],
            )
            for setpoint in setpoints
        ]
        return cls(name, limits, setpoints, duration, *args)

    @staticmethod
    def get_joint_from_urdf(robot, joint_name):
        """Get the name of the robot joint corresponding with the joint in the subgait."""
        for urdf_joint in robot.joints:
            if urdf_joint.name == joint_name:
                return urdf_joint
        return None

    @property
    def duration(self):
        return self._duration

    def set_duration(self, new_duration, rescale=True):
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
    def setpoints(self):
        return self._setpoints

    @setpoints.setter
    def setpoints(self, setpoints):
        self._setpoints = setpoints
        self.interpolate_setpoints()

    def get_setpoints_unzipped(self):
        """Get all the listed attributes of the setpoints."""
        time = []
        position = []
        velocity = []

        for setpoint in self.setpoints:
            time.append(setpoint.time)
            position.append(setpoint.position)
            velocity.append(setpoint.velocity)

        return time, position, velocity

    def validate_joint_transition(self, joint):
        """Validate the ending and starting of this joint to a given joint.

        :param joint:
            the joint of the next subgait (not the previous one)

        :returns:
            True if ending and starting point are identical else False
        """
        if not self._validate_boundary_points():
            return False

        from_setpoint = self.setpoints[-1]
        to_setpoint = joint.setpoints[0]

        if (
            abs(from_setpoint.velocity - to_setpoint.velocity) <= ALLOWED_ERROR
            and abs(from_setpoint.position - to_setpoint.position) <= ALLOWED_ERROR
        ):
            return True

        return False

    def _validate_boundary_points(self):
        """Validate the starting and ending of this joint are at t = 0 and t = duration, or that their speed is zero.

        :returns:
            False if the starting/ending point is (not at 0/duration) and (has nonzero speed), True otherwise
        """
        return (self.setpoints[0].time == 0 or self.setpoints[0].velocity == 0) and (
            self.setpoints[-1].time == round(self.duration, Setpoint.digits)
            or self.setpoints[-1].velocity == 0
        )

    def interpolate_setpoints(self):
        if len(self.setpoints) == 1:
            self.interpolated_position = lambda time: self.setpoints[0].position
            self.interpolated_velocity = lambda time: self.setpoints[0].velocity
            return

        time, position, velocity = self.get_setpoints_unzipped()
        yi = []
        for i in range(0, len(time)):
            yi.append([position[i], velocity[i]])

        # We do a cubic spline here, just like the ros1 joint_trajectory_action_controller,
        # see https://wiki.ros.org/robot_mechanism_controllers/JointTrajectoryActionController
        self.interpolated_position = BPoly.from_derivatives(time, yi)
        self.interpolated_velocity = self.interpolated_position.derivative()

    def get_interpolated_setpoint(self, time):
        if time < 0:
            rospy.logerr("Could not interpolate setpoint at time {0}".format(time))
            return self.setpoint_class(time, self.setpoints[0].position, 0)
        if time > self.duration:
            rospy.logerr("Could not interpolate setpoint at time {0}".format(time))
            return self.setpoint_class(time, self.setpoints[-1].position, 0)

        return self.setpoint_class(
            time,
            float(self.interpolated_position(time)),
            float(self.interpolated_velocity(time)),
        )

    def __getitem__(self, index):
        return self.setpoints[index]

    def __len__(self):
        return len(self.setpoints)

    @staticmethod
    def interpolate_joint_trajectories(base_trajectory, other_trajectory, parameter):
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
                "Not able to safely interpolate because limits are not equal for joints "
                "{0} and {1}".format(base_trajectory.name, other_trajectory.name)
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

        duration = (
            parameter * base_trajectory.duration
            + (1 - parameter) * other_trajectory.duration
        )
        return JointTrajectory(
            base_trajectory.name, base_trajectory.limits, setpoints, duration
        )

    @staticmethod
    def change_order_of_joints_and_setpoints(base_subgait, other_subgait):
        """Change structure from joints which have a list of setpoints to a list of 'ith' setpoints with for each joint.

        This function goes over each joint to get needed setpoints (all first setpoints, all second setpoints..).
        These are placed in list with the correct index, where each entry contains a dictionary with joint name setpoint
        pairs. Also checks whether the joint trajectories are safe to interpolate.

        :param base_subgait: one of the subgaits to reorder
        :param other_subgait: the other subgait to reorder
        :return: The interpolated trajectory
        """
        number_of_setpoints = len(base_subgait.joints[0].setpoints)
        base_setpoints_to_interpolate = [{} for _ in range(number_of_setpoints)]
        other_setpoints_to_interpolate = [{} for _ in range(number_of_setpoints)]
        for setpoint_index in range(number_of_setpoints):
            for base_joint, other_joint in zip(
                sorted(base_subgait.joints, key=lambda joint: joint.name),
                sorted(other_subgait.joints, key=lambda joint: joint.name),
            ):
                base_setpoints_to_interpolate[setpoint_index][
                    base_joint.name
                ] = base_joint.setpoints[setpoint_index]
                other_setpoints_to_interpolate[setpoint_index][
                    other_joint.name
                ] = other_joint.setpoints[setpoint_index]
        return base_setpoints_to_interpolate, other_setpoints_to_interpolate

from march_utility.gait.edge_position import StaticEdgePosition, UnknownEdgePosition
from march_utility.utilities.duration import Duration
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .gait_interface import GaitInterface
from .gait_update import GaitUpdate
from .trajectory_scheduler import TrajectoryCommand


DEFAULT_HOMEGAIT_DURATION = Duration(seconds=3)


class HomeGait(GaitInterface):
    """A standard gait that goes from the unknown state to an idle position."""

    def __init__(
        self, name, position, gait_type, duration: Duration = DEFAULT_HOMEGAIT_DURATION
    ):
        """Initializes an executable home gait with given positions.

        :param str name: Name of the idle position this gait homes to.
                         Will be prefixed with `home_`
        :param dict position: Mapping of joint names to positions
        :param str gait_type: Gait type to use for home gait
        :param Duration duration: Duration of the gait in seconds. Defaults to 3 seconds.
        """
        self._name = "home_{name}".format(name=name)
        self._position = position
        self._gait_type = gait_type
        self._duration = duration
        self._start_time = None
        self._end_time = None

    @property
    def name(self):
        return self._name

    @property
    def subgait_name(self):
        return self._name

    @property
    def duration(self):
        return self._duration

    @property
    def gait_type(self):
        return self._gait_type

    @property
    def starting_position(self):
        return UnknownEdgePosition()

    @property
    def final_position(self):
        return StaticEdgePosition(self._position)

    @property
    def version(self):
        return "home_gait_version"

    def start(self, current_time: Time) -> GaitUpdate:
        """Start the gait.
        Creates a trajectory command to go towards the idle position in the given duration.
        :returns Returns a GaitUpdate that usually contains a TrajectoryCommand.
        """
        self._start_time = current_time
        self._end_time = self._start_time + self._duration
        return GaitUpdate.should_schedule(
            TrajectoryCommand(
                self._get_trajectory_msg(), self._duration, self._name, self._start_time
            )
        )

    def update(self, current_time: Time) -> GaitUpdate:
        """Give an update on the progress of the gait.
        :param current_time: Current time
        :returns Returns a GaitUpdate with only the is_finished set to either true or false.
        """
        if current_time >= self._end_time:
            return GaitUpdate.finished()
        else:
            return GaitUpdate.empty()

    def _get_trajectory_msg(self):
        """
        Constructs a trajectory message that has only one set point to be
        standing still in the idle position after the specified duration.
        :return:
        """
        msg = JointTrajectory()
        msg.joint_names = sorted(self._position.keys())

        point = JointTrajectoryPoint()
        point.time_from_start = self._duration.to_msg()
        point.positions = [self._position[name] for name in msg.joint_names]
        point.velocities = [0.0] * len(msg.joint_names)
        point.accelerations = [0.0] * len(msg.joint_names)
        point.effort = [0.0] * len(msg.joint_names)

        msg.points = [point]
        return msg

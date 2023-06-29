"""Author: Unknown."""

from typing import Optional, Dict, List
from rclpy.time import Time

from march_utility.gait.edge_position import UnknownEdgePosition
from march_utility.utilities.duration import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand

DEFAULT_HOMEGAIT_DURATION = Duration(seconds=3)
ZERO_DURATION = Duration(seconds=0)


class HomeGait():
    """A standard gait that goes from the unknown state to an idle position.

    Args:
        name (str): Name of the idle position this gait homes to. Will be prefixed with `home_`
        position (Dict[str, float]): Mapping of joint names to positions
        gait_type (str): Gait type to use for home gait
        duration (:obj: Duration, optional): Duration of the gait in seconds, defaults to 3 seconds

    Attributes:
        _name (str): Name of the idle position this gait homes to. Will be prefixed with `home_`
        _position (Dict[str, float]): Mapping of joint names to positions
        _gait_type (str): Gait type to use for home gait
        _duration (Duration): Duration of the gait in seconds, defaults to 3 seconds
    """

    def __init__(
            self,
            name: str,
            position: Dict[str, float],
            gait_type: str,
            duration: Duration = DEFAULT_HOMEGAIT_DURATION,
    ):
        """Initializes an executable home gait with given positions."""
        self._name = "home_{name}".format(name=name)
        self._position = position
        self._gait_type = gait_type
        self._duration = duration
        self._scheduled_early = False
        self._start_time = None
        self._end_time = None
        self._starting_position = UnknownEdgePosition()
        self._final_position = self._position
        self._actuating_joint_names = ["left_ankle", "left_hip_aa", "left_hip_fe", "left_knee",
                                       "right_ankle", "right_hip_aa", "right_hip_fe", "right_knee"]

    @property
    def name(self):
        """Returns the name of the gait."""
        return self._name

    @property
    def subgait_name(self):
        """Returns the name of the subgait."""
        return self._name

    @property
    def duration(self):
        """Returns the duration of the subgait."""
        return self._duration

    @property
    def gait_type(self):
        """Returns the type of the gait."""
        return self._gait_type

    @property
    def starting_position(self):
        """Returns the starting position of the gait."""
        return self._starting_position

    @property
    def final_position(self):
        """Returns the final position of the gait."""
        return self._final_position

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        """If the first subgait can be scheduled early."""
        return True

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        """If subsequent subgait can be scheduled early."""
        return True

    @property
    def version(self):
        """Returns the version of the subgait."""
        return "home_gait_version"

    def start(
            self,
            current_time: Time,
            first_subgait_delay: Optional[Duration] = ZERO_DURATION,
    ) -> GaitUpdate:
        """Start the gait.

        Creates a trajectory command to go towards the idle position in the given duration.

        Args:
            current_time (rclpy.Time): current time
            first_subgait_delay (:obj: Duration, optional): delay of the first subgait, defaults to zero
        Returns:
            GaitUpdate: a GaitUpdate that usually contains a TrajectoryCommand.
        """
        if first_subgait_delay is not None and first_subgait_delay > Duration(0):
            self._start_time = current_time + first_subgait_delay
            self._end_time = self._start_time + self._duration
            self._scheduled_early = True
            return GaitUpdate.should_schedule_early(
                TrajectoryCommand(
                    self._get_trajectory_msg(),
                    self._duration,
                    self._name,
                    self._start_time,
                )
            )
        else:
            self._start_time = current_time
            self._end_time = self._start_time + self._duration
            return GaitUpdate.should_schedule(
                TrajectoryCommand(
                    self._get_trajectory_msg(),
                    self._duration,
                    self._name,
                    self._start_time,
                )
            )

    def update(
            self,
            current_time: Time,
            delay: float,
    ) -> GaitUpdate:
        """Give an update on the progress of the gait.

         Args:
            current_time (Time): The current time.
            delay (float): delay with which to schedule the next subgait.

        Returns:
            GaitUpdate: GaitUpdate that may contain a TrajectoryCommand, and any of the
                flags set to true, depending on the state of the Gait.
        """
        if current_time >= self._end_time:
            return GaitUpdate.finished()
        elif self._scheduled_early and current_time > self._start_time:
            return GaitUpdate.subgait_updated()
        else:
            return GaitUpdate.empty()

    def _get_trajectory_msg(self) -> JointTrajectory:
        """Constructs a trajectory msg that has a set point standing still in the idle position after the duration.

        Returns:
            JointTrajectory: message containing joint trajectories
        """
        msg = JointTrajectory()
        msg.joint_names = self._actuating_joint_names

        point = JointTrajectoryPoint()
        point.time_from_start = self._duration.to_msg()
        point.positions = self._position.values
        point.velocities = [0.0] * len(msg.joint_names)
        point.accelerations = [0.0] * len(msg.joint_names)
        point.effort = [0.0] * len(msg.joint_names)

        msg.points = [point]
        return msg

"""Author: Marten Haitjema, MVII"""

from rclpy.time import Time
from rclpy.node import Node

from march_utility.gait.edge_position import EdgePosition, StaticEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    get_joint_names_from_urdf,
    get_limits_robot_from_urdf_for_inverse_kinematics,
)
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.utility_functions import get_position_from_yaml
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait

from march_shared_msgs.msg import FootPosition

FOOT_LOCATION_TIME_OUT = Duration(0.5)


class DynamicSetpointGait(GaitInterface):
    """Gait built up from dynamic setpoints

    :param gait_selection_node: the gait selection node
    :type gait_selection_node: Node
    """

    def __init__(self, gait_selection_node: Node):
        super(DynamicSetpointGait, self).__init__()
        self.gait_selection = gait_selection_node
        self._reset()
        self.joint_names = get_joint_names_from_urdf()
        self._get_soft_limits()

        self.gait_name = "dynamic_walk"

        # Create subscribers and publishers for CoViD
        self.gait_selection.create_subscription(
            FootPosition,
            "/foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            FootPosition,
            "/foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )

        # Assign reconfigurable parameters
        self.update_parameters()

        self.logger = Logger(self.gait_selection, __class__.__name__)

    @property
    def name(self) -> str:
        return self.gait_name

    @property
    def subgait_name(self) -> str:
        # Should return left_swing/right_swing for simulation to work
        return self.subgait_id

    @property
    def version(self) -> str:
        return "v0"

    @property
    def duration(self) -> Duration:
        if self._next_command is not None:
            return self._next_command.duration
        else:
            return None

    @property
    def gait_type(self) -> str:
        # Return gait type based on height of desired foot location
        if self._next_command is not None:
            if (
                self.foot_location.point.y > self.minimum_stair_height
                or self.foot_location.point.y < self.minimum_stair_height
            ):
                return "stairs_like"
            else:
                return "walk_like"
        else:
            return None

    @property
    def starting_position(self) -> EdgePosition:
        return StaticEdgePosition(
            self._setpoint_dict_to_joint_dict(self.start_position)
        )

    @property
    def final_position(self) -> EdgePosition:
        # Beunmethod to fix transitions, should be fixed
        if self._next_command is not None:
            return StaticEdgePosition(
                self._setpoint_dict_to_joint_dict(
                    self.dynamic_subgait.get_final_position()
                )
            )
        else:
            return StaticEdgePosition(
                self._setpoint_dict_to_joint_dict(self.end_position)
            )

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        return True

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        return True

    def _reset(self) -> None:
        """Reset all attributes of the gait"""
        self._should_stop = False
        self._end = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self.subgait_id = "right_swing"
        self._next_command = None

        self._start_is_delayed = True
        self._scheduled_early = False

        self.start_position = self._joint_dict_to_setpoint_dict(
            get_position_from_yaml("stand")
        )
        self.end_position = self.start_position

    DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)

    def start(
        self,
        current_time: Time,
        first_subgait_delay: Duration = DEFAULT_FIRST_SUBGAIT_START_DELAY,
    ) -> GaitUpdate:
        """Starts the gait. The subgait will be scheduled with the delay given
        by first_subgait_delay.

        :param current_time: Time at which the subgait will start
        :type current_time: Time
        :param first_subgait_delay: Delay of first subgait schedule
        :type first_subgait_delay: Duration

        :return: A GaitUpdate containing a TrajectoryCommand
        :rtype: GaitUpdate
        """
        self._reset()
        self.update_parameters()
        self._current_time = current_time
        self._start_time = self._current_time + first_subgait_delay
        self._next_command = self._get_trajectory_command(start=True)
        return GaitUpdate.should_schedule_early(self._next_command)

    DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION = Duration(0)

    def update(
        self,
        current_time: Time,
        early_schedule_duration: Duration = DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION,
    ) -> GaitUpdate:
        """Give an update on the progress of the gait. This function is called
        every cycle of the gait_state_machine.

        Schedules the first subgait when the delay has passed. Starts scheduling
        subsequent subgaits when the previous subgait is within early scheduling
        duration and updates the state machine when the next subgait is started.

        :param current_time: Current time.
        :type current_time: Time
        :param early_schedule_duration: Duration that determines how long ahead the next subgait is planned
        :type early_schedule_duration: Duration

        :return: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
        :rtype: GaitUpdate
        """
        self._current_time = current_time

        if self._start_is_delayed:
            if self._current_time >= self._start_time:
                return self._update_start_subgait()
            else:
                return GaitUpdate.empty()

        if (
            self._current_time >= self._end_time - early_schedule_duration
            and not self._scheduled_early
        ):
            return self._update_next_subgait_early()

        if self._current_time >= self._end_time:
            return self._update_state_machine()

        return GaitUpdate.empty()

    def stop(self) -> bool:
        """Called when the current gait should be stopped"""
        self._should_stop = True
        return True

    def end(self) -> None:
        """Called when the gait is finished"""
        self._next_command = None

    def _update_start_subgait(self) -> GaitUpdate:
        """Update the state machine that the start gait has
        begun. Also updates the start position and the time
        stamps for the next subgait.

        :returns: a GaitUpdate for the state machine
        :rtype: GaitUpdate"""
        self._start_is_delayed = False
        self._update_start_pos()
        self._update_time_stamps(self._next_command.duration)

        return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Already schedule the next subgait with the end time
        of the current subgait as the start time.

        :returns: a GaitUpdate that is empty or contains a trajectory command
        :rtype: GaitUpdate
        """
        self._scheduled_early = True
        self._next_command = self._get_next_command()

        if self._next_command is None:
            return GaitUpdate.empty()

        return GaitUpdate.should_schedule_early(self._next_command)

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the new subgait has begun.
        Also updates the starting position and time stamps for the
        next subgait.

        :returns: a GaitUpdate for the state machine
        :rtype: GaitUpdate
        """
        if self._next_command is None:
            return GaitUpdate.finished()

        self._update_start_pos()
        self._update_time_stamps(self._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()

    def _get_next_command(self) -> TrajectoryCommand:
        """Create the next command, based on what the current subgait is.
        Also checks if the gait has to be stopped. If true, it returns
        a close gait.

        :returns: A TrajectoryCommand for the next subgait
        :rtype: TrajectoryCommand
        """

        if self.subgait_id == "right_swing":
            self.subgait_id = "left_swing"
        elif self.subgait_id == "left_swing":
            self.subgait_id = "right_swing"

        if self._end:
            # If the gait has ended, the next command should be None
            return None
        elif self._should_stop:
            return self._get_trajectory_command(stop=True)
        else:
            return self._get_trajectory_command()

    def _update_start_pos(self) -> None:
        """Update the start position of the next subgait to be
        the last position of the previous subgait."""
        self.start_position = self.dynamic_subgait.get_final_position()

    def _callback_right(self, foot_location: FootPosition) -> None:
        """Update the right foot position with the latest point published
        on the CoViD-topic.

        :param foot_location: a Point containing the x, y and z location
        :type foot_location: FootPosition
        """
        self.foot_location_right = foot_location

    def _callback_left(self, foot_location: FootPosition) -> None:
        """Update the left foot position with the latest point published
        on the CoViD-topic.

        :param foot_location: a Point containing the x, y and z location
        :type foot_location: FootPosition
        """
        self.foot_location_left = foot_location

    def _get_foot_location(self, subgait_id: str) -> FootPosition:
        """Returns the right or left foot position based upon the subgait_id

        :param subgait_id: either right_swing or left_swing
        :type subgait_id: str
        :return: either the left or right foot position or none
        :rtype: FootPosition
        """
        if subgait_id == "left_swing":
            return self.foot_location_left
        elif subgait_id == "right_swing":
            return self.foot_location_right
        else:
            return None

    def _covid_to_gait_magic(self) -> None:
        temp = self.foot_location.point.y
        self.foot_location.point.x = -self.foot_location.point.x
        gait_covid_offset = 0.1
        self.foot_location.point.x -= gait_covid_offset
        self.foot_location.point.y = self.foot_location.point.z + 0.05
        self.foot_location.point.z = temp

    def _get_trajectory_command(self, start=False, stop=False) -> TrajectoryCommand:
        """Return a TrajectoryCommand based on current subgait_id

        :param start: whether it is a start gait or not
        :type start: bool
        :param stop: whether it is a stop gait or not
        :type stop: bool

        :return: TrajectoryCommand with the current subgait and start time.
        :rtype: TrajectoryCommand
        """
        if self._start_is_delayed:
            self._end_time = self._start_time

        if stop:
            self._end = True
            self.logger.info("Stopping dynamic gait.")
        else:
            self.foot_location = self._get_foot_location(self.subgait_id)
            stop = self._check_msg_time(self.foot_location)
            self._covid_to_gait_magic()
            self.logger.info(
                f"Stepping to location ({self.foot_location.point.x}, {self.foot_location.point.y})"
            )

        self.dynamic_subgait = DynamicSubgait(
            self.gait_selection,
            self.start_position,
            self.subgait_id,
            self.joint_names,
            self.foot_location.point,
            self.joint_soft_limits,
            start,
            stop,
        )

        trajectory = self.dynamic_subgait.get_joint_trajectory_msg()
        duration = self.dynamic_subgait.get_duration_scaled_to_height(
            self.dynamic_subgait_duration, self.foot_location.point.y
        )

        return TrajectoryCommand(
            trajectory,
            Duration(duration),
            self.subgait_id,
            self._end_time,
        )

    def _update_time_stamps(self, next_command_duration: Duration) -> None:
        """Update the starting and end time

        :param next_command_duration: Duration of the next command to be scheduled.
        :type next_command_duration: Duration
        """
        self._start_time = self._end_time
        self._end_time = self._start_time + next_command_duration

    def update_parameters(self) -> None:
        """Callback for gait_selection_node when the parameters have been updated."""
        self.dynamic_subgait_duration = self.gait_selection.dynamic_subgait_duration
        self.minimum_stair_height = self.gait_selection.minimum_stair_height

    # UTILITY FUNCTIONS
    @staticmethod
    def _setpoint_dict_to_joint_dict(setpoint_dict: dict) -> dict:
        """Creates a joint_dict from a setpoint_dict.

        :param setpoint_dict: A dictionary containing joint names and setpoints.
        :type: dict

        :returns: A dictionary containing joint names and positions.
        :rtype: dict
        """
        joint_dict = {}
        for name, setpoint in setpoint_dict.items():
            joint_dict[name] = setpoint.position

        return joint_dict

    @staticmethod
    def _joint_dict_to_setpoint_dict(joint_dict: dict) -> dict:
        """Creates a setpoint_dict from a joint_dict.

        :param joint_dict: A dictionary containing joint names and positions.
        :type: dict

        :returns: A dictionary containing joint names and setpoints.
        :rtype: dict
        """
        setpoint_dict = {}
        for name, position in joint_dict.items():
            setpoint_dict[name] = Setpoint(Duration(0), position, 0)
        return setpoint_dict

    def _get_soft_limits(self):
        """Get the limits of all joints in the urdf"""
        self.joint_soft_limits = []
        for joint_name in self.joint_names:
            self.joint_soft_limits.append(
                get_limits_robot_from_urdf_for_inverse_kinematics(joint_name)
            )

    # SAFETY
    def _check_msg_time(self, foot_location: FootPosition) -> bool:
        """Checks if the foot_location given by CoViD is not older than
        FOOT_LOCATION_TIME_OUT."""
        msg_time = Time(
            seconds=foot_location.header.stamp.sec,
            nanoseconds=foot_location.header.stamp.nanosec,
        )
        current_time = Time(
            seconds=self.gait_selection.get_clock().now().seconds_nanoseconds()[0],
            nanoseconds=self.gait_selection.get_clock().now().seconds_nanoseconds()[1],
        )
        time_difference = current_time - msg_time
        self.logger.debug(
            "Time difference between CoViD foot location and current time: "
            f"{time_difference}.",
        )

        if time_difference > FOOT_LOCATION_TIME_OUT:
            self.logger.info(
                "Foot location is more than 0.5 seconds old, time difference is "
                f"{time_difference}. Stopping gait.",
            )
            self._end = True
            return True

        return False

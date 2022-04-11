"""Author: Marten Haitjema, MVII"""

from typing import Optional, Dict
from math import floor
from rclpy.time import Time
from rclpy.node import Node

from march_utility.gait.edge_position import EdgePosition, StaticEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    get_joint_names_from_urdf,
    get_limits_robot_from_urdf_for_inverse_kinematics,
    get_position_from_gait_selection,
    get_position_from_yaml,
)
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
    ShouldStartFromHomestandError,
)

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait
from march_gait_selection.dynamic_interpolation.dynamic_joint_trajectory import NANOSECONDS_TO_SECONDS
from march_shared_msgs.msg import FootPosition, GaitInstruction

FOOT_LOCATION_TIME_OUT = Duration(0.5)
DURATION_INCREASE_FACTOR = 1.5
DURATION_INCREASE_SIZE = 0.25


class DynamicSetpointGait(GaitInterface):
    """Gait built up from dynamic setpoints

    :param gait_selection_node: the gait selection node
    :type gait_selection_node: Node
    """

    def __init__(self, gait_selection_node: Node):
        super(DynamicSetpointGait, self).__init__()
        self.gait_selection = gait_selection_node
        self.logger = Logger(self.gait_selection, __class__.__name__)
        self._trajectory_failed = False

        self.start_position_actuating_joints = get_position_from_gait_selection(self.gait_selection, "stand")
        self.start_position_all_joints = get_position_from_yaml("stand")
        self.home_stand_position_actuating_joints = self.start_position_actuating_joints
        self.home_stand_position_all_joints = self.start_position_all_joints

        self._reset()
        self.joint_names = get_joint_names_from_urdf()
        self._get_soft_limits()

        self.gait_name = "dynamic_walk"

        # Create subscribers and publishers for CoViD
        self.gait_selection.create_subscription(
            FootPosition,
            "/processed_foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            FootPosition,
            "/processed_foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            GaitInstruction,
            "/march/input_device/instruction",
            self._callback_force_unknown,
            DEFAULT_HISTORY_DEPTH,
        )

        # Assign reconfigurable parameters
        self.update_parameters()

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
    def duration(self) -> Optional[Duration]:
        if self._next_command is not None:
            return self._next_command.duration
        else:
            return None

    @property
    def gait_type(self) -> Optional[str]:
        if self._next_command is not None:
            if (
                self.foot_location.processed_point.y > self.minimum_stair_height
                or self.foot_location.processed_point.y < -self.minimum_stair_height
            ):
                return "stairs_like"
            else:
                return "walk_like"
        else:
            return None

    @property
    def starting_position(self) -> EdgePosition:
        return StaticEdgePosition(self.start_position_actuating_joints)

    @property
    def final_position(self) -> EdgePosition:
        try:
            return StaticEdgePosition(
                self._dict_all_joints_to_actuating_joints(self.dynamic_subgait.get_final_position())
            )
        except AttributeError:
            return StaticEdgePosition(self.home_stand_position_actuating_joints)

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        return True

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        return True

    def _reset(self) -> None:
        """Reset all attributes of the gait"""
        if self.start_position_actuating_joints != self.home_stand_position_actuating_joints:
            raise ShouldStartFromHomestandError(self.start_position_actuating_joints)

        self._should_stop = False
        self._end = False

        self._start_time_next_command = None
        self._next_command = None
        self.subgait_id = "right_swing"

        self._start_is_delayed = True
        self._scheduled_early = False

        self.start_position_actuating_joints = get_position_from_gait_selection(self.gait_selection, "stand")
        self.start_position_all_joints = get_position_from_yaml("stand")

        self._trajectory_failed = False

    DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)

    def start(
        self,
        current_time: Time,
        first_subgait_delay: Duration = DEFAULT_FIRST_SUBGAIT_START_DELAY,
    ) -> Optional[GaitUpdate]:
        """Starts the gait. The subgait will be scheduled with the delay given
        by first_subgait_delay.

        :param current_time: Time at which the subgait will start
        :type current_time: Time
        :param first_subgait_delay: Delay of first subgait schedule
        :type first_subgait_delay: Duration

        :return: A GaitUpdate containing a TrajectoryCommand
        :rtype: GaitUpdate
        """
        try:
            self._reset()
        except ShouldStartFromHomestandError as e:
            self.logger.error(e.msg)
            return None
        self.update_parameters()
        self._start_time_next_command = current_time + first_subgait_delay
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
        if self._start_is_delayed:
            if current_time >= self._start_time_next_command:
                return self._update_start_subgait()
            else:
                return GaitUpdate.empty()

        if current_time >= self._start_time_next_command - early_schedule_duration and not self._scheduled_early:
            return self._update_next_subgait_early()

        if current_time >= self._start_time_next_command:
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
        begun. Updates the time stamps for the next subgait.

        :returns: a GaitUpdate for the state machine
        :rtype: GaitUpdate"""
        self._start_is_delayed = False
        self._update_time_stamps(self._next_command.duration)

        return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Already schedule the next subgait with the end time
        of the current subgait as the start time.

        :returns: a GaitUpdate that is empty or contains a trajectory command
        :rtype: GaitUpdate
        """
        self._scheduled_early = True
        self._next_command = self._set_and_get_next_command()

        if self._next_command is None:
            return GaitUpdate.empty()

        return GaitUpdate.should_schedule_early(self._next_command)

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the new subgait has begun.
        Also updates time stamps for the next subgait.

        :returns: a GaitUpdate for the state machine
        :rtype: GaitUpdate
        """
        if self._next_command is None:
            return GaitUpdate.finished()

        self._update_time_stamps(self._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()

    def _set_and_get_next_command(self) -> Optional[TrajectoryCommand]:
        """Create the next command, based on what the current subgait is.
        Also checks if the gait has to be stopped. If true, it returns
        a close gait.

        :returns: A TrajectoryCommand for the next subgait
        :rtype: TrajectoryCommand
        """
        if not self._trajectory_failed:
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
        self.start_position_all_joints = self.dynamic_subgait.get_final_position()
        self.start_position_actuating_joints = self._dict_all_joints_to_actuating_joints(self.start_position_all_joints)

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

    def _get_foot_location(self, subgait_id: str) -> Optional[FootPosition]:
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

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id.

        :param start: whether it is a start gait or not
        :type start: bool
        :param stop: whether it is a stop gait or not
        :type stop: bool

        :return: TrajectoryCommand with the current subgait and start time.
        :rtype: TrajectoryCommand
        """
        if stop:
            self._end = True
            self.logger.info("Stopping dynamic gait.")
        else:
            try:
                self.foot_location = self._get_foot_location(self.subgait_id)
            except AttributeError:
                self.logger.warn("No FootLocation found. Connect the camera or use simulated points.")
                self._end = True
                return None
            stop = self._check_msg_time(self.foot_location)
            self.logger.info(
                f"Stepping to location ({self.foot_location.processed_point.x}, "
                f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
            )

        if start and stop:
            return None

        return self._get_first_feasible_trajectory(start, stop)

    def _get_first_feasible_trajectory(self, start: bool, stop: bool) -> Optional[TrajectoryCommand]:
        """If a subgait is not feasible, it will first try to increase the duration. If it is
        still not feasible, execution of the gait will be stopped.

        :param start: whether it is a start gait or not
        :type start: bool
        :param stop: whether it is a stop gait or not
        :type stop: bool

        :return: TrajectoryCommand with the current subgait and start time.
        :rtype: TrajectoryCommand
        """
        original_duration = self.foot_location.duration
        max_iteration = (
            floor(((original_duration * DURATION_INCREASE_FACTOR - original_duration) / DURATION_INCREASE_SIZE)) - 1
        )
        iteration = 0

        while not self._is_duration_bigger_than_max_duration(original_duration):
            is_final_iteration = iteration == max_iteration
            trajectory_command = self._try_to_get_trajectory_command(
                start,
                stop,
                original_duration,
                iteration,
                is_final_iteration,
            )
            # Return command if current and next step can be made at same duration
            second_step = self._try_to_get_second_step(is_final_iteration)
            if trajectory_command is not None and second_step:
                self._trajectory_failed = False
                self._update_start_pos()
                return trajectory_command
            else:
                self._trajectory_failed = True
                self.foot_location.duration += DURATION_INCREASE_SIZE
            iteration += 1

        # If no feasible subgait can be found, try to execute close gait
        if not start:
            try:
                return self._get_stop_gait()
            except (PositionSoftLimitError, VelocitySoftLimitError) as e:
                self.logger.warn(f"Can not get stop gait. {e.msg}")

        # If close gait is not feasible, stop gait completely
        self._end = True
        return None

    def _try_to_get_trajectory_command(
        self,
        start: bool,
        stop: bool,
        original_duration: float,
        iteration: float,
        is_final_iteration: bool,
    ) -> Optional[TrajectoryCommand]:
        """Try to get a joint_trajectory_msg from the dynamic subgait instance.

        :param start: whether it is a start gait or not
        :type start: bool
        :param stop: whether it is a stop gait or not
        :type stop: bool

        :return: TrajectoryCommand if successful, otherwise None
        :rtype: TrajectoryCommand
        """
        try:
            self.dynamic_subgait = self._create_subgait_instance(
                self.start_position_all_joints, self.subgait_id, start, stop
            )
            trajectory = self.dynamic_subgait.get_joint_trajectory_msg()
            self.logger.debug(
                f"Found trajectory after {iteration + 1} iterations at duration of {self.foot_location.duration}. "
                f"Original duration was {original_duration}."
            )
            return TrajectoryCommand(
                trajectory,
                Duration(self.foot_location.duration),
                self.subgait_id,
                self._start_time_next_command,
            )
        except (PositionSoftLimitError, VelocitySoftLimitError) as e:
            if is_final_iteration:
                self.logger.warn(
                    f"Can not get trajectory after {iteration + 1} iterations. {e.msg} Gait will not be executed."
                )
            return None

    def _try_to_get_second_step(self, is_final_iteration: bool) -> bool:
        """Tries to create the subgait that is one step ahead. If this is not possible,
        the first subgait should not be executed.

        :returns: If the second step can be made
        :rtype: bool
        """
        start_position = self.dynamic_subgait.get_final_position()
        subgait_id = "right_swing" if self.subgait_id == "left_swing" else "left_swing"
        subgait = self._create_subgait_instance(
            start_position,
            subgait_id,
            start=False,
            stop=False,
        )
        try:
            subgait.get_joint_trajectory_msg()
        except (PositionSoftLimitError, VelocitySoftLimitError) as e:
            if is_final_iteration:
                self.logger.warn(f"Second step is not feasible. {e.msg}")
            return False
        return True

    def _get_stop_gait(self) -> Optional[TrajectoryCommand]:
        self._end = True
        subgait = self._create_subgait_instance(
            self.start_position_all_joints,
            self.subgait_id,
            start=False,
            stop=True,
        )
        trajectory = subgait.get_joint_trajectory_msg()
        return TrajectoryCommand(
            trajectory,
            Duration(self.foot_location.duration),
            self.subgait_id,
            self._start_time_next_command,
        )

    def _is_duration_bigger_than_max_duration(self, original_duration: float) -> bool:
        """Returns true if duration is bigger than maximum duration, else false.

        :param original_duration: duration before iterations
        :type original_duration: float
        """
        return self.foot_location.duration >= original_duration * DURATION_INCREASE_FACTOR

    def _create_subgait_instance(
        self,
        start_position: dict,
        subgait_id: str,
        start: bool,
        stop: bool,
    ) -> DynamicSubgait:
        """Create a DynamicSubgait instance

        :param start: whether it is a start gait or not
        :type start: bool
        :param stop: whether it is a stop gait or not
        :type stop: bool
        """
        return DynamicSubgait(
            self.gait_selection,
            start_position,
            subgait_id,
            self.joint_names,
            self.foot_location,
            self.joint_soft_limits,
            start,
            stop,
        )

    def _update_time_stamps(self, next_command_duration: Duration) -> None:
        """Update the starting and end time

        :param next_command_duration: Duration of the next command to be scheduled.
        :type next_command_duration: Duration
        """
        start_time_previous_command = self._start_time_next_command
        self._start_time_next_command = start_time_previous_command + next_command_duration

    def update_parameters(self) -> None:
        """Callback for gait_selection_node when the parameters have been updated."""
        self.minimum_stair_height = self.gait_selection.minimum_stair_height

    def _callback_force_unknown(self, msg: GaitInstruction) -> None:
        if msg.type == GaitInstruction.UNKNOWN:
            self.start_position_all_joints = get_position_from_yaml("stand")
            self.start_position_actuating_joints = self._dict_all_joints_to_actuating_joints(
                self.start_position_all_joints
            )
            self.subgait_id = "right_swing"
            self._trajectory_failed = False

    def _get_soft_limits(self):
        """Get the limits of all joints in the urdf"""
        self.joint_soft_limits = []
        for joint_name in self.joint_names:
            self.joint_soft_limits.append(get_limits_robot_from_urdf_for_inverse_kinematics(joint_name))

    def _dict_all_joints_to_actuating_joints(self, dict_all_joints: Dict[str, float]) -> Dict[str, float]:
        dict_actuating_joints = {}
        for name in self.joint_names:
            dict_actuating_joints[name] = dict_all_joints[name]
        return dict_actuating_joints

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
        readable_time_difference = (
                f"{time_difference.nanoseconds / NANOSECONDS_TO_SECONDS}"
        )
        self.logger.debug(
            f"Time difference between CoViD foot location and current time: {readable_time_difference}.",
        )

        if time_difference > FOOT_LOCATION_TIME_OUT:
            self.logger.warn(
                f"Foot location is more than {FOOT_LOCATION_TIME_OUT} seconds old, time difference is "
                f"{readable_time_difference} seconds. Stopping gait."
            )
            self._end = True
            return True

        return False

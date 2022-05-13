"""Author: Marten Haitjema, MVII."""

from typing import Optional, Dict, Union
from math import floor
from rclpy.time import Time
from copy import copy

from march_utility.gait.edge_position import EdgePosition, StaticEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    get_joint_names_from_urdf,
    get_limits_robot_from_urdf_for_inverse_kinematics,
    get_position_from_yaml,
)
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_utility.utilities.logger import Logger
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
    WrongStartPositionError,
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
    """Gait built up from dynamic setpoints.

    Args:
        gait_selection_node (GaitSelection): the gait selection node

    Attributes:
        gait_selection (GaitSelection): the gait selection node
        home_stand_position_actuating_joints (Dict[str, float]): joint dict of home stand position for only the
            actuating joints
        home_stand_position_all_joints (Dict[str, float]): joint dict of home stand position for all eight joints
        start_position_actuating_joints (Dict[str, float]): start_position of the actuating joints. Home stand if the
            gait has not  started yet, last setpoint of previous step if gait is running
        start_position_all_joints (Dict[str, float): start_position of all eight joints. Home stand if the gait has not
            started yet, last setpoint of previous step if gait is running.
        actuating_joint_names (List[str]): names of the actuating joints in alphabetical order
        all_joint_names (List[srt]): names of all eight joints in alphabetical order
        gait_name (str): name of the gait
        subgait_id (str): either left_swing or right_swing
        logger (Logger): used to log messages to the terminal with the class name as a prefix
        pub_left (Publisher): used to publish the chosen foot position of the left leg
        pub_right (Publisher): used to publish the chosen foot position of the right leg
        minimum_stair_height (float): steps higher or lower than this height will be classified as 'stairs-like'

        _end (bool): whether the gait has ended
        _next_command (Optional[TrajectoryCommand]): TrajectoryCommand that should be scheduled next
        _trajectory_failed (bool): True if step is not feasible (e.g. due to soft limits), else False
        _start_time_next_command (Optional[Union[Duration, Time]]): time at which the next command will be scheduled
        _should_stop (bool): Set to true if the next subgait should be a close gait
    """

    _start_time_next_command: Optional[Union[Duration, Time]]
    _next_command: Optional[TrajectoryCommand]
    _should_stop: bool
    minimum_stair_height: float
    add_push_off: bool
    amount_of_steps: int

    def __init__(self, gait_selection_node):
        super(DynamicSetpointGait, self).__init__()
        self.gait_selection = gait_selection_node
        self.logger = Logger(self.gait_selection, __class__.__name__)
        self._trajectory_failed = False

        self.home_stand_position_actuating_joints = self.gait_selection.get_named_position("stand")
        self.home_stand_position_all_joints = get_position_from_yaml("stand")
        self.start_position_actuating_joints = copy(self.home_stand_position_actuating_joints)
        self.start_position_all_joints = copy(self.home_stand_position_all_joints)

        self._reset()
        self.all_joint_names = self.home_stand_position_all_joints.keys()
        self.actuating_joint_names = get_joint_names_from_urdf()
        self._get_soft_limits()

        self.gait_name = "dynamic_walk"

        # Create subscribers and publishers for CoViD
        self.gait_selection.create_subscription(
            FootPosition,
            "/march/processed_foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            FootPosition,
            "/march/processed_foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )
        self.gait_selection.create_subscription(
            GaitInstruction,
            "/march/input_device/instruction",
            self._callback_force_unknown,
            DEFAULT_HISTORY_DEPTH,
        )
        self.pub_right = self.gait_selection.create_publisher(
            FootPosition,
            "/march/chosen_foot_position/right",
            DEFAULT_HISTORY_DEPTH,
        )
        self.pub_left = self.gait_selection.create_publisher(
            FootPosition,
            "/march/chosen_foot_position/left",
            DEFAULT_HISTORY_DEPTH,
        )

        # Assign reconfigurable parameters
        self.update_parameters()

    @property
    def name(self) -> str:
        """Returns the name of the gait."""
        return self.gait_name

    @property
    def subgait_name(self) -> str:
        """Returns the name of the subgait. Should return left_swing/right_swing for simulation to work."""
        if self._end and "right" in self.subgait_id:
            return "right_close"
        elif self._end and "left" in self.subgait_id:
            return "left_close"
        else:
            return self.subgait_id

    @property
    def version(self) -> str:
        """Returns the version of the subgait."""
        return "v0"

    @property
    def duration(self) -> Optional[Duration]:
        """Returns the duration of the subgait."""
        if self._next_command is not None:
            return self._next_command.duration
        else:
            return None

    @property
    def gait_type(self) -> Optional[str]:
        """Returns the type of gait, for example 'walk_like' or 'sit_like'."""
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
        """Returns the starting position of the subgait as an EdgePosition."""
        return StaticEdgePosition(self.start_position_actuating_joints)

    @property
    def final_position(self) -> EdgePosition:
        """Returns the final position of the subgait as an EdgePosition."""
        try:
            return StaticEdgePosition(
                {name: self.dynamic_subgait.get_final_position()[name] for name in self.actuating_joint_names}
            )
        except AttributeError:
            return StaticEdgePosition(self.home_stand_position_actuating_joints)

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        """Returns if the subgait can be scheduled early."""
        return True

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        """Returns if the first open subgait can be scheduled early."""
        return True

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        if self.start_position_actuating_joints != self.home_stand_position_actuating_joints:
            raise WrongStartPositionError(
                self.home_stand_position_actuating_joints, self.start_position_actuating_joints
            )

        self._should_stop = False
        self._end = False
        self._trajectory_failed = False

        self._start_time_next_command = None
        self._next_command = None
        self.subgait_id = "right_swing"

        self._start_is_delayed = True
        self._scheduled_early = False

        self._trajectory_failed = False

        self.start_position_actuating_joints = self.gait_selection.get_named_position("stand")
        self.start_position_all_joints = get_position_from_yaml("stand")

        self._step_counter = 0

    DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)

    def start(
        self,
        current_time: Time,
        first_subgait_delay: Duration = DEFAULT_FIRST_SUBGAIT_START_DELAY,
    ) -> Optional[GaitUpdate]:
        """Starts the gait. The subgait will be scheduled with the delay given by first_subgait_delay.

        Args:
            current_time (Time): Time at which the subgait will start
            first_subgait_delay (Duration): Delay of first subgait schedule
        Returns:
            GaitUpdate: An optional GaitUpdate containing a TrajectoryCommand if step is feasible
        """
        try:
            self._reset()
        except WrongStartPositionError as e:
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
        """Give an update on the progress of the gait. This function is called every cycle of the gait_state_machine.

        Schedules the first subgait when the delay has passed. Starts scheduling subsequent subgaits when the previous
        subgait is within early scheduling duration and updates the state machine when the next subgait is started.

        Args:
            current_time (Time): Current time
            early_schedule_duration (Duration): Duration that determines how long ahead the next subgait is planned
        Returns:
            GaitUpdate: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
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
        """Called when the current gait should be stopped."""
        self._should_stop = True
        return True

    def end(self) -> None:
        """Called when the gait is finished."""
        self._next_command = None

    def _update_start_subgait(self) -> GaitUpdate:
        """Update the state machine that the start gait has begun. Updates the time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        self._start_is_delayed = False
        self._update_time_stamps(self._next_command.duration)

        return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Already schedule the next subgait with the end time of the current subgait as the start time.

        Returns:
            GaitUpdate: a GaitUpdate that is empty or contains a trajectory command
        """
        self._scheduled_early = True
        self._next_command = self._set_and_get_next_command()

        if self._next_command is None:
            return GaitUpdate.empty()

        return GaitUpdate.should_schedule_early(self._next_command)

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the new subgait has begun. Also updates time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        if self._next_command is None:
            return GaitUpdate.finished()

        self._update_time_stamps(self._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()

    def _set_and_get_next_command(self) -> Optional[TrajectoryCommand]:
        """Create the next command, based on what the current subgait is.

        Also checks if the gait has to be stopped. If true, it returns a close gait.

        Returns:
            TrajectoryCommand: A TrajectoryCommand for the next subgait
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
            return self._get_trajectory_command(stop=self._check_step_count())

    def _update_start_position_gait_state(self) -> None:
        """Update the start position of the next subgait to be the last position of the previous subgait."""
        self.start_position_all_joints = self.dynamic_subgait.get_final_position()
        self.start_position_actuating_joints = {
            name: self.start_position_all_joints[name] for name in self.actuating_joint_names
        }

    def _callback_right(self, foot_location: FootPosition) -> None:
        """Update the right foot position with the latest point published on the CoViD-topic.

        Args:
            foot_location (FootPosition): a Point containing the x, y, and z location
        """
        self.foot_location_right = foot_location

    def _callback_left(self, foot_location: FootPosition) -> None:
        """Update the left foot position with the latest point published on the CoViD-topic.

        Args:
            foot_location (FootPosition): a Point containing the x, y, and z location
        """
        self.foot_location_left = foot_location

    def _get_foot_location(self, subgait_id: str) -> Optional[FootPosition]:
        """Returns the right or left foot position based upon the subgait_id.

        Args:
            subgait_id (str): Either right_swing or left_swing
        Returns:
            FootPosition: either the left or right foot position or none
        """
        if subgait_id == "left_swing":
            return self.foot_location_left
        elif subgait_id == "right_swing":
            return self.foot_location_right
        else:
            return None

    def _publish_chosen_foot_position(self, subgait_id: str, foot_position: FootPosition) -> None:
        """Publish the point to which the step is planned.

        Args:
            subgait_id (str): whether it is a right or left swing
            foot_position (FootPosition): point message to which step is planned
        """
        if subgait_id == "left_swing":
            self.pub_left.publish(foot_position)
        elif subgait_id == "right_swing":
            self.pub_right.publish(foot_position)

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id.

        Args:
            start (:obj: bool, optional): whether` it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time. Returns None if the location found by
                CoViD is too old.
        """
        if stop:
            self._end = True
            self.logger.info("Stopping dynamic gait.")
        else:
            try:
                self.foot_location = self._get_foot_location(self.subgait_id)
                stop = self._is_foot_location_too_old(self.foot_location)
            except AttributeError:
                self.logger.warn("No FootLocation found. Connect the camera or use simulated points.")
                self._end = True
                return None
            if not stop:
                self._publish_chosen_foot_position(self.subgait_id, self.foot_location)
                self.logger.info(
                    f"Stepping to location ({self.foot_location.processed_point.x}, "
                    f"{self.foot_location.processed_point.y}, {self.foot_location.processed_point.z})"
                )

        if start and stop:
            # If it is a start gait and stop is set to true because of the message time, do not return a trajectory.
            return None

        return self._get_first_feasible_trajectory(start, stop)

    def _check_step_count(self) -> bool:
        """Returns True if the gait should stop because it has reached its max step count."""
        if self.amount_of_steps < 1:
            return False
        elif self._step_counter == self.amount_of_steps - 1:
            self._end = True
            self.logger.info("Stopping dynamic gait.")
            return True
        self._step_counter += 1
        return False

    def _get_first_feasible_trajectory(self, start: bool, stop: bool) -> Optional[TrajectoryCommand]:
        """Returns the first trajectory than can be executed.

        If a subgait is not feasible, it will first try to increase the duration. If it is
        still not feasible, execution of the gait will be stopped.

        Args:
            start (:obj: bool, optional): whether` it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
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
            second_step = self._can_get_second_step(is_final_iteration)
            if trajectory_command is not None and second_step:
                self._trajectory_failed = False
                self._update_start_position_gait_state()
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
        iteration: int,
        is_final_iteration: bool,
    ) -> Optional[TrajectoryCommand]:
        """Try to get a joint_trajectory_msg from the dynamic subgait instance.

        Args:
            start (bool): whether it is a start gait
            stop (bool): whether it is a stop gait
            original_duration (float): original duration of the gait as set in the GaitPreprocessor
            iteration (int): current iteration over the velocity
            is_final_iteration (bool): True if current iteration equals the maximum amount of iterations
        Returns:
            TrajectoryCommand: optional command if successful, otherwise None
        """
        try:
            self.dynamic_subgait = self._create_subgait_instance(
                self.start_position_all_joints, self.subgait_id, start, stop
            )
            trajectory = self.dynamic_subgait.get_joint_trajectory_msg(self.add_push_off)
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

    def _can_get_second_step(self, is_final_iteration: bool) -> bool:
        """Tries to create the subgait that is one step ahead.

        If this is not possible, the first subgait should not be executed.

        Args:
            is_final_iteration (bool): True if current iteration equals the maximum amount of iterations
        Returns:
            bool: True if second step can be made, otherwise false
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
            subgait.get_joint_trajectory_msg(self.add_push_off)
        except (PositionSoftLimitError, VelocitySoftLimitError) as e:
            if is_final_iteration:
                self.logger.warn(f"Second step is not feasible. {e.msg}")
            return False
        return True

    def _get_stop_gait(self) -> Optional[TrajectoryCommand]:
        """Returns a TrajectoryCommand containing a stop gait.

        Returns:
            TrajectoryCommand: command containing a stop gait
        """
        self._end = True
        subgait = self._create_subgait_instance(
            self.start_position_all_joints,
            self.subgait_id,
            start=False,
            stop=True,
        )
        trajectory = subgait.get_joint_trajectory_msg(self.add_push_off)
        return TrajectoryCommand(
            trajectory,
            Duration(self.foot_location.duration),
            self.subgait_id,
            self._start_time_next_command,
        )

    def _is_duration_bigger_than_max_duration(self, original_duration: float) -> bool:
        """Returns true if duration is bigger than maximum duration, else false.

        Args:
            original_duration (float): duration before iterations
        Returns:
            bool: True if current duration is bigger than max allowed duration, else False
        """
        return self.foot_location.duration >= original_duration * DURATION_INCREASE_FACTOR

    def _create_subgait_instance(
        self,
        start_position: Dict[str, float],
        subgait_id: str,
        start: bool,
        stop: bool,
    ) -> DynamicSubgait:
        """Create a DynamicSubgait instance.

        Args:
            start_position (Dict[str, float]): dict containing joint_names and positions of the joint as floats
            subgait_id (str): either 'left_swing' or 'right_swing'
            start (bool): whether it is a start gait or not
            stop (bool): whether it is a stop gait or not
        Returns:
            DynamicSubgait: DynamicSubgait instance for the desired step
        """
        return DynamicSubgait(
            self.gait_selection,
            self.home_stand_position_all_joints,
            start_position,
            subgait_id,
            self.actuating_joint_names,
            self.foot_location,
            self.joint_soft_limits,
            start,
            stop,
        )

    def _update_time_stamps(self, next_command_duration: Duration) -> None:
        """Update the starting and end time.

        Args:
            next_command_duration (Duration): duration of the next command to be scheduled
        """
        start_time_previous_command = self._start_time_next_command
        self._start_time_next_command = start_time_previous_command + next_command_duration

    def update_parameters(self) -> None:
        """Callback for gait_selection_node when the parameters have been updated."""
        self.minimum_stair_height = self.gait_selection.minimum_stair_height
        self.add_push_off = self.gait_selection.add_push_off
        self.amount_of_steps = self.gait_selection.amount_of_steps

    def _get_soft_limits(self):
        """Get the limits of all joints in the urdf."""
        self.joint_soft_limits = []
        for joint_name in self.actuating_joint_names:
            self.joint_soft_limits.append(get_limits_robot_from_urdf_for_inverse_kinematics(joint_name))

    def _callback_force_unknown(self, msg: GaitInstruction) -> None:
        """Reset start position to home stand after force unknown.

        Args:
            msg (GaitInstruction): message containing a gait_instruction from the IPD
        """
        if msg.type == GaitInstruction.UNKNOWN:
            self.start_position_all_joints = copy(self.home_stand_position_all_joints)
            self.start_position_actuating_joints = {
                name: self.start_position_all_joints[name] for name in self.actuating_joint_names
            }
            self.subgait_id = "right_swing"
            self._trajectory_failed = False

    def _is_foot_location_too_old(self, foot_location: FootPosition) -> bool:
        """Checks if the foot_location given by CoViD is not older than FOOT_LOCATION_TIME_OUT.

        Args:
            foot_location (FootPosition): FootPosition message that should be checked
        Returns:
            bool: True if message is not more than 0.5 seconds old, else False
        """
        msg_time = Time(
            seconds=foot_location.header.stamp.sec,
            nanoseconds=foot_location.header.stamp.nanosec,
        )
        current_time = Time(
            seconds=self.gait_selection.get_clock().now().seconds_nanoseconds()[0],
            nanoseconds=self.gait_selection.get_clock().now().seconds_nanoseconds()[1],
        )
        time_difference = current_time - msg_time
        readable_time_difference = f"{time_difference.nanoseconds / NANOSECONDS_TO_SECONDS}"
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

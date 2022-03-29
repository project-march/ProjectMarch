"""Author: Marten Haitjema, MVII"""

from typing import Optional, Dict
from math import floor
from rclpy.time import Time

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
from march_utility.exceptions.gait_exceptions import (
    PositionSoftLimitError,
    VelocitySoftLimitError,
    ShouldStartFromHomestandError,
)

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait

from march_shared_msgs.msg import FootPosition, GaitInstruction

FOOT_LOCATION_TIME_OUT = Duration(0.5)
DURATION_INCREASE_FACTOR = 2
DURATION_INCREASE_SIZE = 0.25


class DynamicSetpointGait(GaitInterface):
    """Gait built up from dynamic setpoints.

    Args:
        gait_selection_node (GaitSelection): the gait selection node

    Attributes:
        gait_selection (GaitSelection): the gait selection node
        home_stand_position (Dict[str, Setpoint]): setpoint_dict of home stand position
        start_position (Dict[str, Setpoint]): start_position of gait. Home stand if the gait has not started yet,
            last setpoint of previous step if gait is running
        end_position (Dict[str, Setpoint]): setpoint_dict of end position
        joint_names (List[str]): names of the joints
        gait_name (str): name of the gait
        subgait_id (str): either left_swing or right_swing

        _end (bool): whether the gait has ended
        _next_command (Optional[TrajectoryCommand]): TrajectoryCommand that should be scheduled next
        _trajectory_failed (bool): True if step is not feasible (e.g. due to soft limits), else False
    """

    _start_time: Optional[Duration]
    _current_time: Optional[Time]
    _next_command: Optional[TrajectoryCommand]
    _should_stop: bool
    _minimum_stair_height: float

    def __init__(self, gait_selection_node):
        super(DynamicSetpointGait, self).__init__()
        self.gait_selection = gait_selection_node
        self.home_stand_position = self._joint_dict_to_setpoint_dict(get_position_from_yaml("stand"))
        self.start_position = self.home_stand_position
        self.end_position = self.home_stand_position
        self._trajectory_failed = False
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
    def duration(self) -> Optional[Duration]:
        if self._next_command is not None:
            return self._next_command.duration
        else:
            return None

    @property
    def gait_type(self) -> Optional[str]:
        if self._next_command is not None:
            if (
                self.foot_location.point.y > self._minimum_stair_height
                or self.foot_location.point.y < -self._minimum_stair_height
            ):
                return "stairs_like"
            else:
                return "walk_like"
        else:
            return None

    @property
    def starting_position(self) -> EdgePosition:
        return StaticEdgePosition(self._setpoint_dict_to_joint_dict(self.start_position))

    @property
    def final_position(self) -> EdgePosition:
        try:
            return StaticEdgePosition(self._setpoint_dict_to_joint_dict(self.dynamic_subgait.get_final_position()))
        except AttributeError:
            return StaticEdgePosition(self._setpoint_dict_to_joint_dict(self.end_position))

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        return True

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        return True

    def _reset(self) -> None:
        """Reset all attributes of the gait"""
        if self.start_position != self.home_stand_position:
            raise ShouldStartFromHomestandError

        self._should_stop = False
        self._end = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._next_command = None
        self.subgait_id = "right_swing"

        self._start_is_delayed = True
        self._scheduled_early = False

        self._trajectory_failed = False

    DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)

    def start(
        self,
        current_time: Time,
        first_subgait_delay: Duration = DEFAULT_FIRST_SUBGAIT_START_DELAY,
    ) -> Optional[GaitUpdate]:
        """Starts the gait. The subgait will be scheduled with the delay given
        by first_subgait_delay.

        Args:
            current_time (Time): Time at which the subgait will start
            first_subgait_delay (Duration): Delay of first subgait schedule
        Returns:
            GaitUpdate: An optional GaitUpdate containing a TrajectoryCommand if step is feasible
        """
        try:
            self._reset()
        except ShouldStartFromHomestandError:
            self.logger.error("Cannot start the gait from a position that is not homestand.")
            return None
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

        Args:
            current_time (Time): Current time
            early_schedule_duration (Duration): Duration that determines how long ahead the next subgait is planned
        Returns:
            GaitUpdate: GaitUpdate containing TrajectoryCommand when finished, else empty GaitUpdate
        """
        self._current_time = current_time

        if self._start_is_delayed:
            if self._current_time >= self._start_time:
                return self._update_start_subgait()
            else:
                return GaitUpdate.empty()

        if self._current_time >= self._end_time - early_schedule_duration and not self._scheduled_early:
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
        begun. Updates the time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        self._start_is_delayed = False
        self._update_time_stamps(self._next_command.duration)

        return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Already schedule the next subgait with the end time
        of the current subgait as the start time.

        Returns:
            GaitUpdate: a GaitUpdate that is empty or contains a trajectory command
        """
        self._scheduled_early = True
        self._next_command = self._get_next_command()

        if self._next_command is None:
            return GaitUpdate.empty()

        return GaitUpdate.should_schedule_early(self._next_command)

    def _update_state_machine(self) -> GaitUpdate:
        """Update the state machine that the new subgait has begun.
        Also updates time stamps for the next subgait.

        Returns:
            GaitUpdate: a GaitUpdate for the state machine
        """
        if self._next_command is None:
            return GaitUpdate.finished()

        self._update_time_stamps(self._next_command.duration)
        self._scheduled_early = False

        return GaitUpdate.subgait_updated()

    def _get_next_command(self) -> Optional[TrajectoryCommand]:
        """Create the next command, based on what the current subgait is.
        Also checks if the gait has to be stopped. If true, it returns
        a close gait.

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
            return self._get_trajectory_command()

    def _update_start_pos(self) -> None:
        """Update the start position of the next subgait to be
        the last position of the previous subgait."""
        self.start_position = self.dynamic_subgait.get_final_position()

    def _callback_right(self, foot_location: FootPosition) -> None:
        """Update the right foot position with the latest point published
        on the CoViD-topic.

        Args:
            foot_location (FootPosition): a Point containing the x, y, and z location
        """
        self.foot_location_right = foot_location

    def _callback_left(self, foot_location: FootPosition) -> None:
        """Update the left foot position with the latest point published
        on the CoViD-topic.

        Args:
            foot_location (FootPosition): a Point containing the x, y, and z location
        """
        self.foot_location_left = foot_location

    def _get_foot_location(self, subgait_id: str) -> Optional[FootPosition]:
        """Returns the right or left foot position based upon the subgait_id

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

    def _get_trajectory_command(self, start=False, stop=False) -> Optional[TrajectoryCommand]:
        """Return a TrajectoryCommand based on current subgait_id.

        Args:
            start (:obj: bool, optional): whether` it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        if self._start_is_delayed:
            self._end_time = self._start_time

        if stop:
            self._end = True
            self.logger.info("Stopping dynamic gait.")
        else:
            self.foot_location = self._get_foot_location(self.subgait_id)
            stop = self._check_msg_time(self.foot_location)
            self.logger.debug(
                f"Stepping to location ({self.foot_location.point.x}, {self.foot_location.point.y}, "
                f"{self.foot_location.point.z})"
            )

        return self._get_first_feasible_trajectory(start, stop)

    def _get_first_feasible_trajectory(self, start: bool, stop: bool) -> Optional[TrajectoryCommand]:
        """If a subgait is not feasible, it will first try to increase the duration. If it is
        still not feasible, execution of the gait will be stopped.

        Args:
            start (:obj: bool, optional): whether` it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: command with the current subgait and start time
        """
        original_duration = self.foot_location.duration
        second_step = False
        while not self._is_duration_bigger_than_max_duration(original_duration):
            trajectory_command = self._try_to_get_trajectory_command(start, stop, original_duration)
            # Return command if current and next step can be made at same duration
            second_step = self._try_to_get_second_step()
            if trajectory_command is not None and second_step:
                self._trajectory_failed = False
                self._update_start_pos()
                return trajectory_command
            else:
                self._trajectory_failed = True
                self.foot_location.duration += DURATION_INCREASE_SIZE

        if second_step is False:
            self.logger.warn("Not possible to perform second step.")

        # If no feasible subgait can be found, try to execute close gait
        if not start:
            try:
                return self._get_stop_gait()
            except (PositionSoftLimitError, VelocitySoftLimitError):
                # If close gait is not feasible, stop gait completely
                self.logger.warn("Not possible to perform close gait.")

        self._end = True
        self._get_next_command()
        return None

    def _try_to_get_trajectory_command(
        self,
        start: bool,
        stop: bool,
        original_duration: float,
    ) -> Optional[TrajectoryCommand]:
        """Try to get a joint_trajectory_msg from the dynamic subgait instance.

        Args:
            start (:obj: bool, optional): whether` it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            TrajectoryCommand: optional command if successful, otherwise None
        """
        iteration = floor((self.foot_location.duration - original_duration) / DURATION_INCREASE_SIZE)
        try:
            self.dynamic_subgait = self._create_subgait_instance(self.start_position, self.subgait_id, start, stop)
            trajectory = self.dynamic_subgait.get_joint_trajectory_msg()
            self.logger.debug(
                f"Found trajectory after {iteration} iterations at duration of {self.foot_location.duration}. "
                f"Original duration was {original_duration}."
            )
            return TrajectoryCommand(
                trajectory,
                Duration(self.foot_location.duration),
                self.subgait_id,
                self._end_time,
            )
        except PositionSoftLimitError as e:
            if self._is_duration_bigger_than_max_duration(original_duration):
                self.logger.warn(
                    f"Joint {e.joint_name} will still be outside of soft limits after "
                    f"{iteration} iterations. Position: {e.position}, soft limits: "
                    f"[{e.lower_limit}, {e.upper_limit}]. Gait will not be executed."
                )
            return None
        except VelocitySoftLimitError as e:
            if self._is_duration_bigger_than_max_duration(original_duration):
                self.logger.warn(
                    f"Joint {e.joint_name} will still be outside of velocity limits, after "
                    f"{iteration} iterations. Velocity: {e.velocity}, velocity limit: {e.velocity}. "
                    "Gait will not be executed."
                )
            return None

    def _try_to_get_second_step(self) -> bool:
        """Tries to create the subgait that is one step ahead. If this is not possible,
        the first subgait should not be executed.

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
            subgait.get_joint_trajectory_msg()
        except (PositionSoftLimitError, VelocitySoftLimitError):
            return False
        return True

    def _get_stop_gait(self) -> Optional[TrajectoryCommand]:
        """Returns a TrajectoryCommand containing a stop gait

        Returns:
            TrajectoryCommand: command containing a stop gait
        """
        self._end = True
        subgait = self._create_subgait_instance(
            self.start_position,
            self.subgait_id,
            start=False,
            stop=True,
        )
        trajectory = subgait.get_joint_trajectory_msg()
        return TrajectoryCommand(
            trajectory,
            Duration(self.foot_location.duration),
            self.subgait_id,
            self._end_time,
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
        start_position: dict,
        subgait_id: str,
        start: bool,
        stop: bool,
    ) -> DynamicSubgait:
        """Create a DynamicSubgait instance

        Args:
            start (:obj: bool, optional): whether it is a start gait or not, default False
            stop (:obj: bool, optional): whether it is a stop gait or not, default False
        Returns:
            DynamicSubgait: DynamicSubgait instance for the desired step
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

        Args:
            next_command_duration (Duration): duration of the next command to be scheduled
        """
        self._start_time = self._end_time
        self._end_time = self._start_time + next_command_duration

    def update_parameters(self) -> None:
        """Callback for gait_selection_node when the parameters have been updated."""
        self._minimum_stair_height = self.gait_selection.minimum_stair_height

    def _callback_force_unknown(self, msg: GaitInstruction) -> None:
        """Reset start position to home stand after force unknown

        Args:
            msg (GaitInstruction): message containing a gait_instruction from the IPD
        """
        if msg.type == GaitInstruction.UNKNOWN:
            self.start_position = self._joint_dict_to_setpoint_dict(get_position_from_yaml("stand"))
            self.subgait_id = "right_swing"
            self._trajectory_failed = False

    # UTILITY FUNCTIONS
    @staticmethod
    def _setpoint_dict_to_joint_dict(setpoint_dict: dict) -> Dict[str, float]:
        """Creates a joint_dict from a setpoint_dict.

        Args:
            setpoint_dict (dict): A dictionary containing joint names and setpoints
        Returns:
            dict: A dictionary containing joint names and positions
        :param setpoint_dict: A dictionary containing joint names and setpoints.
        :type: dict
        """
        joint_dict = {}
        for name, setpoint in setpoint_dict.items():
            joint_dict[name] = setpoint.position

        return joint_dict

    @staticmethod
    def _joint_dict_to_setpoint_dict(joint_dict: dict) -> Dict[str, Setpoint]:
        """Creates a setpoint_dict from a joint_dict.

        Args:
            joint_dict: A dictionary containing joint names and positions
        Returns:
            dict: A dictionary containing joint names and setpoints
        """
        setpoint_dict = {}
        for name, position in joint_dict.items():
            setpoint_dict[name] = Setpoint(Duration(0), position, 0)
        return setpoint_dict

    def _get_soft_limits(self):
        """Get the limits of all joints in the urdf"""
        self.joint_soft_limits = []
        for joint_name in self.joint_names:
            self.joint_soft_limits.append(get_limits_robot_from_urdf_for_inverse_kinematics(joint_name))

    # SAFETY
    def _check_msg_time(self, foot_location: FootPosition) -> bool:
        """Checks if the foot_location given by CoViD is not older than
        FOOT_LOCATION_TIME_OUT.

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
        self.logger.debug(
            f"Time difference between CoViD foot location and current time: {time_difference}.",
        )

        if time_difference > FOOT_LOCATION_TIME_OUT:
            self.logger.warn(
                "Foot location is more than 0.5 seconds old, time difference is " f"{time_difference}. Stopping gait.",
            )
            self._end = True
            return True

        return False

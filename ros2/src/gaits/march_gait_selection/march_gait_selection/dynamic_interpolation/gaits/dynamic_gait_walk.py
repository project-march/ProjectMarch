"""Author: Marten Haitjema, MVII."""

from typing import Optional, Dict, Union
from rclpy.time import Time
from copy import copy
from rclpy.node import Node

from march_utility.gait.edge_position import EdgePosition, StaticEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import (
    get_joint_names_from_urdf,
    get_limits_robot_from_urdf_for_inverse_kinematics,
    get_position_from_yaml,
)
from march_utility.exceptions.gait_exceptions import (
    WrongStartPositionError,
)

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand

from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)
from march_gait_selection.dynamic_interpolation.point_handlers.point_handler import PointHandler
from sensor_msgs.msg import JointState


class DynamicGaitWalk(GaitInterface):
    """Gait built up from dynamic setpoints.

    Args:
        node (Node): the gait selection node

    Attributes:
        trajectory_command_factory (TrajectoryCommandFactory): the trajectory command handler used to create trajectory commands
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
        _logger (rclpy.logger): used to log messages to the terminal with the class name as a prefix
        minimum_stair_height (float): steps higher or lower than this height will be classified as 'stairs-like'
        start_time_next_command (Optional[Union[Duration, Time]]): time at which the next command will be scheduled
        amount_of_steps (int): the amount of steps the gait makes before closing the gait

        node (Node): the gait selection node
        _point_handler (CameraPointHandler): the camera points handler used to handle communication with vision
        _end (bool): whether the gait has ended
        _next_command (Optional[TrajectoryCommand]): TrajectoryCommand that should be scheduled next
        _should_stop (bool): Set to true if the next subgait should be a close gait
        _has_gait_started (bool): Whether the start time of the gait has passed
    """

    start_time_next_command: Optional[Union[Duration, Time]]
    minimum_stair_height: float
    add_push_off: bool
    amount_of_steps: int
    start_position_all_joints: Dict[str, float]
    start_position_actuating_joints: Dict[str, float]
    _next_command: Optional[TrajectoryCommand]
    _should_stop: bool

    def __init__(self, name: str, node: Node, point_handler: PointHandler):
        super(DynamicGaitWalk, self).__init__()
        self.node = node
        self._logger = node.get_logger().get_child(__class__.__name__)
        self._point_handler = point_handler
        self.trajectory_command_factory = TrajectoryCommandFactory(
            gait=self,
            point_handler=self._point_handler,
        )

        self.actuating_joint_names = get_joint_names_from_urdf()
        self.all_joint_names = get_joint_names_from_urdf(return_fixed_joints=True)
        self.home_stand_position_all_joints = get_position_from_yaml("stand")
        self.home_stand_position_actuating_joints = {
            name: self.home_stand_position_all_joints[name] for name in self.actuating_joint_names
        }

        self._set_start_position_to_home_stand()
        self._reset()
        self._get_soft_limits()
        self.gait_name = name
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
                self._point_handler.get_foot_location(self.subgait_id).processed_point.y > self.minimum_stair_height
                or self._point_handler.get_foot_location(self.subgait_id).processed_point.y < -self.minimum_stair_height
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
                {name: self.trajectory_command_factory.final_position[name] for name in self.actuating_joint_names}
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

    @property
    def requires_dynamic_stop(self) -> bool:
        """Return whether this gait needs a dynamic stop.

        This means that the gait does not end in home_stand, but in another random (dynamic) position.
        """
        return False

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        if self.start_position_actuating_joints != self.home_stand_position_actuating_joints:
            raise WrongStartPositionError(
                self.home_stand_position_actuating_joints, self.start_position_actuating_joints
            )

        self._should_stop = False
        self._end = False

        self.start_time_next_command = None
        self._next_command = None
        self.subgait_id = "right_swing"

        self._has_gait_started = False
        self._scheduled_early = False
        self._step_counter = 0

        self._set_start_position_to_home_stand()

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
            self._logger.error(f"{e}")
            return None
        self.update_parameters()
        self.start_time_next_command = current_time + first_subgait_delay
        self._next_command = self.trajectory_command_factory.get_trajectory_command(
            self.subgait_id, self.start_position_all_joints, start=True
        )
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
        if current_time >= self.start_time_next_command and not self._has_gait_started:
            self._has_gait_started = True
            return self._update_start_subgait()

        elif (
            current_time >= self.start_time_next_command - early_schedule_duration
            and not self._scheduled_early
            and self._has_gait_started
        ):
            self._scheduled_early = True
            return self._update_next_subgait_early()

        elif current_time >= self.start_time_next_command:
            return self._update_state_machine()

        else:
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
        self._update_time_stamps(self._next_command.duration)
        return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Already schedule the next subgait with the end time of the current subgait as the start time.

        Returns:
            GaitUpdate: a GaitUpdate that is empty or contains a trajectory command
        """
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
        """Get the next command, based on what the current subgait is.

        Also checks if the gait has to be stopped. If true, it returns a close gait.

        Returns:
            TrajectoryCommand: A TrajectoryCommand for the next subgait
        """
        self._set_subgait_id()
        if self._end:
            # If the gait has ended, the next command should be None
            return None
        elif self._should_stop:
            return self.trajectory_command_factory.get_trajectory_command(
                self.subgait_id,
                self.start_position_all_joints,
                stop=True,
            )
        else:
            return self.trajectory_command_factory.get_trajectory_command(
                self.subgait_id, self.start_position_all_joints, stop=self._check_step_count()
            )

    def _set_subgait_id(self) -> None:
        """Switch subgait id to the opposite side."""
        if self.subgait_id == "right_swing":
            self.subgait_id = "left_swing"
        elif self.subgait_id == "left_swing":
            self.subgait_id = "right_swing"

    def update_start_position_gait_state(self) -> None:
        """Update the start position of the next subgait to be the last position of the previous subgait."""
        self.start_position_all_joints = self.trajectory_command_factory.final_position
        self.start_position_actuating_joints = {
            name: self.start_position_all_joints[name] for name in self.actuating_joint_names
        }

    def _update_start_position_idle_state(self, joint_state: JointState) -> None:
        """Update the start position of the next subgait to be the last position of the previous gait."""
        for i, name in enumerate(self.all_joint_names):
            self.start_position_all_joints[name] = joint_state.position[i]
        self.start_position_actuating_joints = {
            name: self.start_position_all_joints[name] for name in self.actuating_joint_names
        }

    def _check_step_count(self) -> bool:
        """Returns True if the gait should stop because it has reached its max step count."""
        if self.amount_of_steps < 1:
            return False
        elif self._step_counter == self.amount_of_steps - 1:
            self._end = True
            self._logger.info("Stopping dynamic gait.")
            return True
        self._step_counter += 1
        return False

    def _update_time_stamps(self, next_command_duration: Duration) -> None:
        """Update the starting and end time.

        Args:
            next_command_duration (Duration): duration of the next command to be scheduled
        """
        start_time_previous_command = self.start_time_next_command
        self.start_time_next_command = start_time_previous_command + next_command_duration

    def update_parameters(self) -> None:
        """Callback for gait_node when the parameters have been updated."""
        self.trajectory_command_factory.update_parameter()
        self.minimum_stair_height = self.node.minimum_stair_height
        self.add_push_off = self.node.add_push_off
        self.amount_of_steps = self.node.amount_of_steps

    def _get_soft_limits(self):
        """Get the limits of all joints in the urdf."""
        self.joint_soft_limits = []
        for joint_name in self.actuating_joint_names:
            self.joint_soft_limits.append(get_limits_robot_from_urdf_for_inverse_kinematics(joint_name))

    def set_state_to_unknown(self) -> None:
        """Reset start position to home stand after force unknown."""
        self._set_start_position_to_home_stand()
        self.subgait_id = "right_swing"

    def _set_start_position_to_home_stand(self) -> None:
        """Sets the starting position to home_stand."""
        self.start_position_all_joints = copy(self.home_stand_position_all_joints)
        self.start_position_actuating_joints = {
            name: self.start_position_all_joints[name] for name in self.actuating_joint_names
        }
        self.trajectory_command_factory.start_position_all_joints = self.start_position_all_joints
        self.trajectory_command_factory.start_position_actuating_joints = self.start_position_actuating_joints

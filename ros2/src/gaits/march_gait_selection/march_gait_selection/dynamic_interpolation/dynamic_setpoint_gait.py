from rclpy.time import Time

from march_utility.gait.edge_position import EdgePosition, StaticEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.utility_functions import get_joint_names_from_urdf
from march_utility.gait.setpoint import Setpoint
from march_utility.utilities.utility_functions import get_position_from_yaml

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait

from geometry_msgs.msg import Point


class DynamicSetpointGait(GaitInterface):
    """Gait built up from dynamic setpoints"""

    def __init__(self, gait_selection_node):
        super(DynamicSetpointGait, self).__init__()
        self._should_stop = False
        self._end = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._next_command = None

        self.start_position = self._joint_dict_to_setpoint_dict(
            get_position_from_yaml("stand")
        )
        self.end_position = self.start_position

        self.joint_names = get_joint_names_from_urdf()
        self.gait_name = "dynamic_walk"

        self._start_is_delayed = True
        self._scheduled_early = False

        # Create subscribers for CoViD topic
        self.gait_selection = gait_selection_node
        self.gait_selection.create_subscription(
            Point,
            "/foot_position/right",
            self._callback_right,
            10,
        )
        self.gait_selection.create_subscription(
            Point,
            "/foot_position/left",
            self._callback_left,
            10,
        )

        # Assign reconfigurable parameters
        self.update_parameters()

    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        # Should return left_swing/right_swing for simulation to work
        return self.subgait_id

    @property
    def version(self):
        return ""

    @property
    def duration(self):
        if self._next_command is not None:
            return self._next_command.duration
        else:
            return None

    @property
    def gait_type(self):
        # Return gait type based on height of desired foot location
        if self._next_command is not None:
            if (
                self.foot_location.y > self.minimum_stair_height
                or self.foot_location.y < self.minimum_stair_height
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

    def _reset(self):
        """Reset all attributes of the gait"""
        self._should_stop = False
        self._end = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._next_command = None

        self._start_is_delayed = True
        self._scheduled_early = False

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
        self.subgait_id = "right_swing"
        self._first_subgait_delay = first_subgait_delay
        self._start_time = self._current_time + first_subgait_delay
        self._next_command = self._get_trajectory_command()
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

        # If the start is delayed, wait untill the start time has passed
        if self._start_is_delayed:
            if self._current_time >= self._start_time:
                self._start_is_delayed = False
                # Update start position and time stamps for the next gait after
                # the start gait has been scheduled
                self._update_start_pos()
                self._update_time_stamps(self._next_command.duration)
                return GaitUpdate.subgait_updated()
            else:
                return GaitUpdate.empty()

        # If we are within the early schedule duration AND have not scheduled yet,
        # already schedule the next subgait
        if (
            self._current_time >= self._end_time - early_schedule_duration
            and not self._scheduled_early
        ):
            self._scheduled_early = True
            self._next_command = self._get_next_command()

            if self._next_command is None:
                return GaitUpdate.empty()

            return GaitUpdate.should_schedule_early(self._next_command)

        # If the current gait has reached its end time, update the state
        # machine and reset the early schedule attributes
        if self._current_time >= self._end_time:
            if self._next_command is None:
                return GaitUpdate.finished()

            self._update_start_pos()
            self._update_time_stamps(self._next_command.duration)
            self._scheduled_early = False

            return GaitUpdate.subgait_updated()

        return GaitUpdate.empty()

    def stop(self) -> bool:
        """Called when the current gait should be stopped"""
        self._should_stop = True
        return True

    def end(self):
        """Called when the gait is finished"""
        self._next_command = None

    def _get_next_command(self):
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

    def _update_start_pos(self):
        """Update the start position of the next subgait to be
        the last position of the previous subgait."""
        self.start_position = self.dynamic_subgait.get_final_position()

    def _callback_right(self, foot_position: Point):
        """Update the right foot position with the latest point published
        on the CoViD-topic.
        """
        self.foot_position_right = foot_position

    def _callback_left(self, foot_position: Point):
        """Update the left foot position with the latest point published
        on the CoViD-topic."""
        self.foot_position_left = foot_position

    def _get_foot_position(self, subgait_id):
        """Returns the right or left foot position based upon the subgait_id"""
        if subgait_id == "left_swing":
            return self.foot_position_left
        elif subgait_id == "right_swing":
            return self.foot_position_right
        else:
            return None

    def _get_trajectory_command(self, stop=False) -> TrajectoryCommand:
        """Return a TrajectoryCommand based on current subgait_id

        :return: TrajectoryCommand with the current subgait and start time.
        :rtype: TrajectoryCommand
        """
        if self._start_is_delayed:
            self._end_time = self._start_time

        if stop:
            self.foot_location.x = 0  # m
            self.foot_location.y = 0  # m
            self._end = True
        else:
            self.foot_location = self._get_foot_position(self.subgait_id)

        self.dynamic_subgait = DynamicSubgait(
            self.dynamic_subgait_duration,
            self.middle_point_fraction,
            self.start_position,
            self.subgait_id,
            self.joint_names,
            self.foot_location.x,
            self.foot_location.y,
        )

        trajectory = self.dynamic_subgait.get_joint_trajectory_msg()

        return TrajectoryCommand(
            trajectory,
            Duration(self.dynamic_subgait_duration),
            self.subgait_id,
            self._end_time,
        )

    def _update_time_stamps(self, next_command_duration):
        """Update the starting and end time

        :param next_command_duration: Duration of the next command to be scheduled.
        :type next_command_duration: Duration
        """
        self._start_time = self._end_time
        self._end_time = self._start_time + next_command_duration

    def update_parameters(self):
        self.dynamic_subgait_duration = self.gait_selection.dynamic_subgait_duration
        self.middle_point_fraction = self.gait_selection.middle_point_fraction
        self.middle_point_height = self.gait_selection.middle_point_height
        self.minimum_stair_height = self.gait_selection.minimum_stair_height

        self._logger(
            "Parameters updated. "
            f"duration: {self.dynamic_subgait_duration}, "
            f"fraction: {self.middle_point_fraction}, "
            f"height: {self.middle_point_height}, "
            f"stairs: {self.minimum_stair_height}"
        )

    # UTILITY FUNCTIONS
    def _setpoint_dict_to_joint_dict(self, setpoint_dict):
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

    def _joint_dict_to_setpoint_dict(self, joint_dict):
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

    def _logger(self, message):
        """Publish a message on the gait_selection_node logger
        with DYNAMIC_SETPOINT_GAIT as a prefix"""
        self.gait_selection.get_logger().info("DYNAMIC_SETPOINT_GAIT: " f"{message}")

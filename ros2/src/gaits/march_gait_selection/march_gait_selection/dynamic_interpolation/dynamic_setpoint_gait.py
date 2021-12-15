from rclpy.time import Time

from march_utility.gait.edge_position import EdgePosition, StaticEdgePosition
from march_utility.utilities.duration import Duration
from march_utility.gait.setpoint import Setpoint

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_gait_selection.dynamic_interpolation.dynamic_subgait import DynamicSubgait


class DynamicSetpointGait(GaitInterface):
    """Gait built up from dynamic setpoints"""

    def __init__(self):
        super(DynamicSetpointGait, self).__init__()
        self._should_stop = False
        self._is_transitioning = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._current_command = None
        self._next_command = None

        self.start_position = {
            "left_ankle": Setpoint(Duration(0), 0.0, 0),
            "left_hip_aa": Setpoint(Duration(0), 0.0349, 0),
            "left_hip_fe": Setpoint(Duration(0), -0.1745, 0),
            "left_knee": Setpoint(Duration(0), 0.0, 0),
            "right_ankle": Setpoint(Duration(0), 0.0, 0),
            "right_hip_aa": Setpoint(Duration(0), 0.0349, 0),
            "right_hip_fe": Setpoint(Duration(0), -0.1745, 0),
            "right_knee": Setpoint(Duration(0), 0.0, 0),
        }
        self.gait_name = "dynamic_walk_v0"

    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        # Should return left_swing/right_swing for simulation to
        # Can be retrieved from dynamic_subgait
        return self.subgait_id

    @property
    def version(self):
        return "dynamic_subgait_v0"

    @property
    def duration(self):
        if self._current_command is not None:
            return self._current_command.duration
        else:
            return None

    @property
    def gait_type(self):
        # For now only take walk-like gaits
        if self._current_command is not None:
            return "walk_like"
        else:
            return None

    @property
    def starting_position(self) -> EdgePosition:
        return StaticEdgePosition(self.setpoint_dict_to_joint_dict(self.start_position))

    @property
    def final_postion(self) -> EdgePosition:
        return StaticEdgePosition(
            self.setpoint_dict_to_joint_dict(self.dynamic_subgait.get_final_position())
        )

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        return False

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        return False

    def _reset(self):
        """Reset all attributes of the gait"""
        self._should_stop = False
        self._is_transitioning = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._current_command = None
        self._next_command = None

    def start(self, current_time: Time) -> GaitUpdate:
        """Starts the gait"""
        self._current_time = current_time
        self.subgait_id = "right_swing"
        self._start_time = self._current_time
        self._current_command = self.subgait_id_to_trajectory_command()
        return GaitUpdate.should_schedule(self._current_command)

    def update(self, current_time: Time) -> GaitUpdate:
        """Give an update on the progress of the gait"""
        self._current_time = current_time
        if self._current_time >= self._end_time:
            return self._update_next_subgait()

        return GaitUpdate.empty()

    def _update_next_subgait(self) -> GaitUpdate:
        """Update the next subgait

        If the current subgait is left_swing, the next subgait should be
        right_swing, and vice versa.

        :return: optional trajectory_command, is_finished
        """
        next_command = self._get_next_command()
        self._current_command = next_command

        return GaitUpdate.should_schedule(next_command)

    def _get_next_command(self):
        """Check if gait should be stopped. If not, schedule next command"""
        if self._should_stop:
            # SHOULD RETURN A STOP GAIT
            return None

        if self.subgait_id == "right_swing":
            self.subgait_id = "left_swing"
        elif self.subgait_id == "left_swing":
            self.subgait_id = "right_swing"

        return self.subgait_id_to_trajectory_command()

    def stop(self) -> bool:
        """Called when the current gait should be stopped"""
        self._should_stop = True
        return True

    def end(self):
        """Called when the gait is finished"""
        self._current_command = None

    def update_start_pos(self):
        self.start_position = self.dynamic_subgait.get_final_position()

    def setpoint_dict_to_joint_dict(self, setpoint_dict):
        position = []
        for setpoint in setpoint_dict:
            position.append(setpoint_dict[setpoint].position)
        joints = [
            "left_ankle",
            "left_knee",
            "left_hip_fe",
            "left_hip_aa",
            "right_hip_aa",
            "right_hip_fe",
            "right_knee",
            "right_ankle",
        ]
        jointdict = {}
        for i in range(len(position)):
            jointdict.update({joints[i]: position[i]})

        return jointdict

    def subgait_id_to_trajectory_command(self) -> TrajectoryCommand:
        """Construct a TrajectoryCommand from the current subgait_id

        :return: TrajectoryCommand with the current subgait and start time.
        """
        desired_ankle_x = 0.20  # m
        desired_ankle_y = 0.03  # m
        subgait_duration = 1.5
        mid_point_frac = 0.33

        self.dynamic_subgait = DynamicSubgait(
            subgait_duration,
            mid_point_frac,
            self.start_position,
            self.subgait_id,
            desired_ankle_x,
            position_y=desired_ankle_y,
        )

        trajectory = self.dynamic_subgait.to_joint_trajectory_msg()

        # Update the starting position for the next command
        self.update_start_pos()
        self._update_time_stamps(Duration(subgait_duration))

        return TrajectoryCommand(
            trajectory,
            Duration(subgait_duration),
            "dynamic_joint_trajectory",
            self._start_time,
        )

    def _update_time_stamps(self, next_command_duration):
        """Update the starting and end time

        :param next_command: Next command to be scheduled
        """
        if self._end_time is None:
            self._start_time = self._current_time
        else:
            self._start_time = self._end_time
        self._end_time = self._start_time + next_command_duration

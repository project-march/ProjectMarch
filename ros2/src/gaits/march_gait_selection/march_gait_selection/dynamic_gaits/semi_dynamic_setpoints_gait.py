from copy import deepcopy
from typing import Optional

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_utility.gait.subgait import Subgait
from march_utility.gait.subgait_graph import SubgaitGraph
from march_utility.utilities.duration import Duration
from rclpy.time import Time

SHOULD_NOT_FREEZE_FIRST_SECS = Duration(seconds=0.3)
DEFAULT_SEDY_FREEZE_DURATION = Duration(seconds=3)


class SemiDynamicSetpointsGait(SetpointsGait):
    """A semi-dynamic version of the setpoints gait, implements a freeze functionality.

    Args:
        gait_name (str): Name of the gait
        subgaits (dict): Mapping of names to subgait instances
        graph (SubgaitGraph): Mapping of subgait names transitions

    Attributes:
        _should_freeze (bool): True if the gait should freeze
        _is_frozen (bool): True if the gait is currently frozen
        _freeze_duration (Duration): how long the gait should be frozen
        _freeze_position (Dict[str, float]): dict containing joint names and positions
    """

    _current_time: Time
    _current_subgait: Subgait

    def __init__(self, gait_name: str, subgaits: dict, graph: SubgaitGraph):
        super(SemiDynamicSetpointsGait, self).__init__(
            f"dynamic_{gait_name}", subgaits, graph
        )
        self._should_freeze = False
        self._is_frozen = False
        self._freeze_duration = Duration(0)
        self._freeze_position = None

    @property
    def can_freeze(self) -> bool:
        """Returns whether the gait has the ability to freeze. This is meant
        for noticing the step height and ending the subgait earlier. This is
        therefore not possible during the first second of the subgait, to
        prevent accidental freezing."""
        return not (self.elapsed_time < SHOULD_NOT_FREEZE_FIRST_SECS or self._should_freeze or self._is_frozen)

    @property
    def can_be_scheduled_early(self) -> bool:
        return False

    @property
    def elapsed_time(self) -> Duration:
        return Duration.from_ros_duration(self._current_time - self._start_time)

    def freeze(self, duration: Duration = DEFAULT_SEDY_FREEZE_DURATION):
        """
        If the subgait can freeze it will freeze for the given duration, this
        will later be changed to start the next subgait more dynamically
        after the short freeze

        Args:
            duration (Duration): How long to freeze in the current position
        """
        if self.can_freeze:
            self._should_freeze = True
            self._freeze_duration = duration

    def update(self, current_time: Time, *_) -> GaitUpdate:
        """Give an update on the progress of the gait.
        If the current subgait is still running, this does nothing.
        If the gait should be stopped, this will be done
        If the current subgait is done, it will start the next subgait

        Args:
            current_time (Time): Current time
        Returns:
            GaitUpdate: Returns a GaitUpdate that may contain a TrajectoryCommand, and any of the
                flags set to true, depending on the state of the Gait.
        """
        self._current_time = current_time
        if self._should_freeze:
            return self._execute_freeze()

        # If the current subgait is not finished, no new trajectory is necessary
        if current_time < self._end_time:
            return GaitUpdate.empty()

        # If the subgait is finished, and it was frozen, execute the subgait after freeze
        if self._is_frozen:
            self._current_subgait = self._subgait_after_freeze
            self._is_frozen = False
            self._update_time_stamps(self._current_subgait)
            return GaitUpdate.should_schedule(self._command_from_current_subgait())

        return self._update_next_subgait()

    def _execute_freeze(self) -> GaitUpdate:
        """
        Freezes the subgait, currently this means that there is a new subgait
        started of the given freeze duration which ends at the current position.
        If this happens in the middle of a subgait, it plans the rest of the
        original subgait after the freeze.

        Returns:
            GaitUpdate: Returns a GaitUpdate
        """
        self._freeze_position = self._position_after_time()
        self._previous_subgait = self._current_subgait.subgait_name
        self._subgait_after_freeze = self.subgait_after_freeze()
        self._current_subgait = self._freeze_subgait()
        self._should_freeze = False
        self._is_frozen = True
        self._update_time_stamps(self._current_subgait)
        return GaitUpdate.should_schedule(self._command_from_current_subgait())

    def subgait_after_freeze(self) -> Subgait:
        """
        Generates the subgait that should be executed after the freezing.

        Returns:
            Subgait: The subgait to execute after the freeze
        """
        if self._current_time < self._end_time:
            # The subgait after freeze is a copy of the current gait from
            # the current time point
            subgait_after_freeze = deepcopy(self._current_subgait)
            for joint in subgait_after_freeze:
                joint.from_begin_point(self.elapsed_time, self._freeze_position[joint.name])
            subgait_after_freeze.duration -= self.elapsed_time
            subgait_after_freeze.subgait_name = "freeze_subgait_after"
            # Add the subgait after the freeze to the subgait graph
            self.graph.graph[subgait_after_freeze.subgait_name] = {
                "to": self.graph[(self._previous_subgait, self.graph.TO)]
            }
            return subgait_after_freeze
        else:
            # If the current subgait was already done,
            # go to the next subgait after freeze
            return self.subgaits[self.graph[(self._previous_subgait, self.graph.TO)]]

    def _freeze_subgait(self) -> Subgait:
        """
        Generates a subgait of the freeze duration based on the current position.

        Returns:
            Subgait: A subgait to freeze in current position
        """
        new_dict = {
            "description": "A subgait that stays in the same position",
            "duration": self._freeze_duration.nanoseconds,
            "gait_type": self._current_subgait.gait_type,
            "joints": {
                joint.name: [
                    {
                        "position": self._freeze_position[joint.name],
                        "time_from_start": self._freeze_duration.nanoseconds,
                        "velocity": 0,
                    }
                ]
                for joint in self._current_subgait.joints
            },
        }

        # freeze subgait
        return Subgait.from_dict(
            robot=self._current_subgait.robot,
            subgait_dict=new_dict,
            gait_name=self.gait_name,
            subgait_name="freeze",
            version="Only version, generated from code",
        )

    def _position_after_time(self, elapsed_time: Optional[Duration] = None) -> dict:
        """
        The position that the exoskeleton should be in after the elapsed
        time.

        Args:
            elapsed_time (:obj: Time, optional): Time that has elapsed, if None uses the elapsed time of
                the current subgait
        Returns:
            dict: dict with joints and joint positions
        """
        if elapsed_time is None:
            elapsed_time = self.elapsed_time
        return {joint.name: joint.get_interpolated_setpoint(elapsed_time).position for joint in self._current_subgait}

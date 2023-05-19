"""Author: Marco Bak MVIII."""

from typing import Optional, Dict
from urdf_parser_py.urdf import Robot

from march_gait_selection.gaits.transition_subgait import TransitionSubgait
from march_utility.exceptions.gait_exceptions import GaitError
from march_utility.gait.gait import Gait
from march_utility.gait.edge_position import StaticEdgePosition, EdgePosition
from march_utility.gait.subgait import Subgait
from march_utility.gait.subgait_graph import SubgaitGraph
from march_utility.utilities.duration import Duration
from rclpy.time import Time

from march_gait_selection.state_machine.gait_update import GaitUpdate
from march_gait_selection.state_machine.gait_interface import GaitInterface
from march_gait_selection.state_machine.state_machine_input import TransitionRequest
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand


class SetpointsGait(GaitInterface, Gait):
    """The standard gait built up from setpoints.

    Args:
        gait_name (str): Name of the gait
        subgaits (dict): Mapping of names to subgait instances
        graph (SubgaitGraph): Mapping of subgait names to transitions

    Attributes:
        _start_time (Optional[Duration]): time at which the gait should start
        _end_time (Optional[Duration]): time at which the gait ends
        _current_time (Optional[Time]): current time
        _current_subgait (Subgait): current subgait
        _should_stop (bool): whether the gait should stop
        _transition_to_subgait (Dict[Subgait]): ???
        _start_is_delayed (bool): whether the start is delayed (early schedule for start gait)
        _scheduled_early (bool): whether the next step is already scheduled early or not
    """

    def __init__(self, gait_name: str, subgaits: Dict[str, Subgait], graph: SubgaitGraph):
        super(SetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._current_subgait = None
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._start_is_delayed = False
        self._scheduled_early = False
        # Keep track of next subgait for early scheduling
        self._next_subgait = None

    @property
    def name(self):
        """Returns the name of the gait."""
        return self.gait_name

    @property
    def subgait_name(self):
        """Returns the name of the subgait."""
        if self._current_subgait is not None:
            return self._current_subgait.subgait_name
        else:
            return ""

    @property
    def version(self):
        """Returns the version of the subgait."""
        if self._current_subgait is not None:
            return self._current_subgait.version
        else:
            return None

    @property
    def duration(self):
        """Returns the duration of the subgait."""
        if self._current_subgait is not None:
            return self._current_subgait.duration
        else:
            return None

    @property
    def gait_type(self):
        """Returns the type of gait, for example 'walk_like' or 'sit_like'."""
        if self._current_subgait is not None:
            return self._current_subgait.gait_type
        else:
            return None

    @property
    def starting_position(self) -> EdgePosition:
        """Returns the starting position of the subgait."""
        return StaticEdgePosition(self.subgaits[self.graph.start_subgaits()[0]].starting_position)

    @property
    def final_position(self) -> EdgePosition:
        """Returns the final position of the subgait."""
        return StaticEdgePosition(self.subgaits[self.graph.end_subgaits()[0]].final_position)

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        """Returns if subsequent subgaits can be scheduled early."""
        return True

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        """Returns if the first subgait can be scheduled early."""
        return True

    def _reset(self) -> None:
        """Reset all attributes of the gait."""
        self._current_subgait = None
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False

        self._start_time = None
        self._end_time = None
        self._current_time = None

        self._start_is_delayed = False
        self._scheduled_early = False
        self._next_subgait = None

    DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)

    def start(
            self,
            current_time: Time,
            first_subgait_delay: Optional[Duration] = DEFAULT_FIRST_SUBGAIT_START_DELAY,
    ) -> GaitUpdate:
        """Start the gait.

        Sets current subgait to the first subgait, resets the time and generates the first trajectory command.
        May optionally delay the first subgait.

        Args:
            current_time (Time): current time
            first_subgait_delay (:obj: Duration, optional): optional first subgait delay
        Returns:
            TrajectoryCommand: A TrajectoryCommand message with the trajectory of the first subgait.
        """
        self._reset()
        self._current_time = current_time
        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._next_subgait = self._current_subgait

        # Delay first subgait if duration is greater than zero
        if first_subgait_delay > Duration(0):
            self._start_is_delayed = True
            self._update_time_stamps(self._current_subgait, first_subgait_delay)
            return GaitUpdate.should_schedule_early(self._command_from_current_subgait())
        else:
            self._start_is_delayed = False
            self._update_time_stamps(self._current_subgait)
            return GaitUpdate.should_schedule(self._command_from_current_subgait())

    DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION = Duration(0)

    def update(
            self,
            current_time: Time,
            early_schedule_duration: Optional[Duration] = DEFAULT_EARLY_SCHEDULE_UPDATE_DURATION,
    ) -> GaitUpdate:
        """Give an update on the progress of the gait.

        - If the start was delayed, and we have passed the start time,
            we are now actually starting the gait.
            Hence, the is_new_subgait flag should be set to true.
        - If the previous subgait ended, schedule a new one.
        - If we haven't scheduled early yet, and we are within early_schedule_duration of
            the end time, then schedule a new subgait early.
        - Else return nothing.

        Args:
            current_time (Time): current_time
            early_schedule_duration (:obj: Duration, optional): Optional duration to schedule early
        Returns:
            GaitUpdate: GaitUpdate that may contain a TrajectoryCommand, and any of the
                flags set to true, depending on the state of the Gait.
        """
        self._current_time = current_time

        if self._start_is_delayed:
            if self._current_time >= self._start_time:
                # Reset start delayed flag and update first subgait
                self._start_is_delayed = False
                return GaitUpdate.subgait_updated()
            else:
                return GaitUpdate.empty()

        if self._current_time >= self._end_time:
            return self._update_next_subgait()

        if (
                early_schedule_duration > Duration(0)
                and not self._scheduled_early
                and self._current_time >= self._end_time - early_schedule_duration
        ):
            return self._update_next_subgait_early()
        return GaitUpdate.empty()

    def _update_next_subgait(self) -> GaitUpdate:
        """Update the next subgait.

        Behaves differently based on whether a new subgait has been scheduled early.
        First the next subgait is determined, which is based on the status of transitioning
        and whether the subgait has to be stopped.

        Then if the next subgait is not None the current subgait is replaced by the
        next subgait, and all appropriate values are updated.

        If a new subgait hasn't been scheduled early, this function also returns a
        TrajectoryCommand.

        Returns:
            GaitUpdate: optional trajectory_command, is_finished
        """
        if self._transition_to_subgait is not None and not self._is_transitioning:
            # We should schedule a transition subgait
            self._is_transitioning = True
            next_subgait = self._make_transition_subgait()
        elif self._transition_to_subgait is not None and self._is_transitioning:
            # A transition subgait has ended
            next_subgait = self.subgaits.get(self._transition_to_subgait.subgait_name)
            self._transition_to_subgait = None
            self._is_transitioning = False
        elif self._scheduled_early:
            # We scheduled early and already determined the next subgait
            next_subgait = self._next_subgait
        else:
            # We determine the next subgait with the subgait graph
            next_subgait = self._next_graph_subgait()

        # If there is no next subgait, then we are finished
        if next_subgait is None:
            return GaitUpdate.finished()

        # Update subgait and timestamps
        self._update_time_stamps(next_subgait)
        self._current_subgait = next_subgait

        # Schedule the next subgait if we haven't already scheduled early
        if not self._scheduled_early:
            return GaitUpdate.should_schedule(self._command_from_current_subgait())
        else:
            # Reset early schedule attributes
            self._scheduled_early = False
            self._next_subgait = None
            return GaitUpdate.subgait_updated()

    def _update_next_subgait_early(self) -> GaitUpdate:
        """Update the next subgait.

        First the next subgait is determined, which is based on the status of transitioning
        and whether the subgait has to be stopped.

        If the next subgait is not None a TrajectoryCommand containing that subgait is returned.

        When updating the next subgait early, we don't set the current_subgait or a new start time.
        Instead, we only send a schedule command and let self._update_next_subgait()
        deal with clean up after the previous subgait has actually finished.

        Returns:
            GaitUpdate: GaitUpdate containing optional trajectory_command
        """
        self._scheduled_early = True
        if self._transition_to_subgait is not None and not self._is_transitioning:
            # We should schedule a transition subgait
            next_subgait = self._make_transition_subgait()
        elif self._transition_to_subgait is not None and self._is_transitioning:
            next_subgait = self.subgaits.get(self._transition_to_subgait.subgait_name)
        else:
            next_subgait = self._next_graph_subgait()

        self._next_subgait = next_subgait
        # If there is no subgait, return None and False for is_finished
        if next_subgait is None:
            return GaitUpdate.empty()

        return GaitUpdate.should_schedule_early(TrajectoryCommand.from_subgait(next_subgait, self._end_time))

    def _next_graph_subgait(self) -> Optional[Subgait]:
        """Get the next subgait from the graph.

        If the setpoints should stop it looks up the stop attribute in the gait graph.
        Otherwise, it looks up the to attribute in the gait graph.

        Returns:
            Subgait: optional subgait that is possible from gait_graph
        """
        next_subgait_name = None
        if self._should_stop:
            next_subgait_name = self.graph[(self._current_subgait.subgait_name, self.graph.STOP)]
        if next_subgait_name is None:
            next_subgait_name = self.graph[(self._current_subgait.subgait_name, self.graph.TO)]

        if next_subgait_name == self.graph.END:
            return None
        else:
            return self.subgaits[next_subgait_name]

    def transition(self, transition_request: TransitionRequest) -> bool:
        """Request to transition between two subgaits with increasing or decreasing size of the step.

        Args:
            transition_request (TransitionRequest): request for transitions
        Returns:
            bool: True if transition can be executed, else False
        """
        if self._is_transitioning or self._should_stop:
            return False

        if transition_request == TransitionRequest.DECREASE_SIZE:
            name = self.graph[(self._current_subgait.subgait_name, self.graph.DECREASE_SIZE)]
        elif transition_request == TransitionRequest.INCREASE_SIZE:
            name = self.graph[(self._current_subgait.subgait_name, self.graph.INCREASE_SIZE)]
        else:
            return False

        if name is not None:
            self._transition_to_subgait = self.subgaits[name]
            return True
        return False

    def stop(self) -> bool:
        """Called when the current gait should be stopped.

        Returns:
            bool: whether the stopping was successful.
        """
        if self._can_stop():
            self._should_stop = True
            return True
        else:
            return False

    def _can_stop(self) -> bool:
        """Determine if the gait can stop at the current moment."""
        return self.graph.is_stoppable() and not self._is_transitioning and self._transition_to_subgait is None

    def end(self) -> None:
        """Called when the gait has finished."""
        self._current_subgait = None

    def set_subgait_versions(self, robot: Robot, gait_directory: str, version_map: Dict[str, str]) -> None:
        """Change the versions of the subgaits.

        Args:
            robot (Robot): the robot model used
            gait_directory (str): the directory where the gaits are located
            version_map (Dict[str, str]): the map with the new versions to use
        Raises:
            GaitError: if subgait version is changed during execution
        """
        if self._current_subgait is None:
            super(SetpointsGait, self).set_subgait_versions(robot, gait_directory, version_map)
        else:
            raise GaitError("Cannot change subgait version while gait is being executed")

    def _make_transition_subgait(self) -> TransitionSubgait:
        """Creates the transition subgait from the next subgait to the subgait stored in _transition_to_subgait.

        Returns:
            TransitionSubgait: the transition subgait
        """
        old_subgait = self.subgaits[self.graph[(self._current_subgait.subgait_name, self.graph.TO)]]
        new_subgait = self.subgaits[self.graph[(self._transition_to_subgait.subgait_name, self.graph.TO)]]
        return TransitionSubgait.from_subgaits(
            old_subgait,
            new_subgait,
            "{s}_transition".format(s=self._transition_to_subgait.subgait_name),
        )

    def _command_from_current_subgait(self) -> TrajectoryCommand:
        """Construct a TrajectoryCommand from the current subgait.

        Returns:
            TrajectoryCommand: TrajectoryCommand with the current subgait and start time.
        """
        return TrajectoryCommand.from_subgait(self._current_subgait, self._start_time)

    DEFAULT_FIRST_SUBGAIT_UPDATE_TIMESTAMPS_DELAY = Duration(0)

    def _update_time_stamps(
            self,
            next_subgait: Subgait,
            first_subgait_delay: Optional[Duration] = DEFAULT_FIRST_SUBGAIT_UPDATE_TIMESTAMPS_DELAY,
    ) -> None:
        """Update the starting and end time.

        Args:
            next_subgait (Subgait): the next subgait that will be scheduled
            first_subgait_delay (:obj: Duration, optional): optional first subgait delay, default is zero.
        """
        if not self._scheduled_early or self._end_time is None:
            self._start_time = self._current_time + first_subgait_delay
        else:
            self._start_time = self._end_time
        self._end_time = self._start_time + next_subgait.duration

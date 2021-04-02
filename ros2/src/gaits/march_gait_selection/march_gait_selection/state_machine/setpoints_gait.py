from typing import Optional, Tuple

from march_gait_selection.dynamic_gaits.transition_subgait import TransitionSubgait
from march_utility.exceptions.gait_exceptions import GaitError
from march_utility.gait.gait import Gait
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration
from rclpy.time import Time

from .trajectory_scheduler import TrajectoryCommand

from .gait_interface import GaitInterface
from .state_machine_input import TransitionRequest


class SetpointsGait(GaitInterface, Gait):
    """ The standard gait built up from setpoints """

    def __init__(self, gait_name, subgaits, graph):
        super(SetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._current_subgait = None
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._start_time = None
        self._end_time = None
        self._scheduled_early = False

    @property
    def name(self):
        return self.gait_name

    @property
    def subgait_name(self):
        if self._current_subgait is not None:
            return self._current_subgait.subgait_name
        else:
            return None

    @property
    def version(self):
        if self._current_subgait is not None:
            return self._current_subgait.version
        else:
            return None

    @property
    def duration(self):
        if self._current_subgait is not None:
            return self._current_subgait.duration
        else:
            return None

    @property
    def gait_type(self):
        if self._current_subgait is not None:
            return self._current_subgait.gait_type
        else:
            return None

    @property
    def starting_position(self):
        return self.subgaits[self.graph.start_subgaits()[0]].starting_position

    @property
    def final_position(self):
        return self.subgaits[self.graph.end_subgaits()[0]].final_position

    @property
    def can_be_scheduled_early(self) -> bool:
        return True

    def start(self, current_time: Time):
        """
        Start the gait, sets current subgait to the first subgait, resets the
        time and generates the first trajectory command.
        :return: A TrajectoryCommand message with the trajectory of the first subgait.
        """
        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._scheduled_early = False
        self._update_time_stamps(
            current_time, self._current_subgait, self._current_subgait
        )
        return self._command_from_current_subgait()

    def update(
        self,
        current_time: Time,
        early_schedule_duration: Optional[Duration] = None,
    ) -> Tuple[Optional[TrajectoryCommand], bool]:
        """
        Update the progress of the gait, should be called regularly.
        If the previous subgait ended, schedule a new one.
        If we haven't scheduled early yet, and we are within early_schedule_duration of
        the end time, then schedule a new subgait early.
        Else return nothing.
        :param current_time: Current time
        :return: optional trajectory_command, is_finished
        """
        if current_time >= self._end_time:
            return self._update_next_subgait(current_time)
        if (
            early_schedule_duration is not None
            and not self._scheduled_early
            and current_time >= self._end_time - early_schedule_duration
        ):
            return self._update_next_subgait_early(), False
        return None, False

    def _update_next_subgait(
        self, current_time: Time
    ) -> Tuple[Optional[TrajectoryCommand], bool]:
        """Update the next subgait.

        Behaves differently based on whether a new subgait has been scheduled early.
        First the next subgait is determined, which is based on the status of transitioning
        and whether the subgait has to be stopped.

        Then if the next subgait is not None the current subgait is replaced by the
        next subgait, and all appriopriate values are updated.

        If a new subgait hasn't been scheduled early, this function also returns a
        TrajectoryCommand.

        :param current_time: Current time
        :return: optional trajectory_command, is_finished
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
        else:
            next_subgait = self._next_graph_subgait()

        # If there is no next subgait, then we are finished
        if next_subgait is None:
            return None, True

        # Update subgait and timestamps
        self._update_time_stamps(current_time, self._current_subgait, next_subgait)
        self._current_subgait = next_subgait

        # Schedule the next subgait if we haven't already scheduled early
        if not self._scheduled_early:
            command = self._command_from_current_subgait()
        else:
            command = None

        self._scheduled_early = False
        return command, False

    def _update_next_subgait_early(self) -> Optional[TrajectoryCommand]:
        """Update the next subgait.

        First the next subgait is determined, which is based on the status of transitioning
        and whether the subgait has to be stopped.

        If the next subgait is not None a TrajectoryComamnd containing that subgait is returned.

        When updating the next subgait early, we don't set the current_subgait or a new start time.
        Instead we only send a schedule command and let self._update_next_subgait()
        deal with clean up after the previous subgait has actually finished.

        :param current_time: Current time
        :return: optional trajectory_command
        """
        self._scheduled_early = True
        if self._transition_to_subgait is not None and not self._is_transitioning:
            # We should schedule a transition subgait
            next_subgait = self._make_transition_subgait()
        elif self._transition_to_subgait is not None and self._is_transitioning:
            next_subgait = self.subgaits.get(self._transition_to_subgait.subgait_name)
        else:
            next_subgait = self._next_graph_subgait()

        # If there is no subgait, return None and False for is_finished
        if next_subgait is None:
            return None
        return TrajectoryCommand.from_subgait(next_subgait, self._end_time)

    def _next_graph_subgait(self) -> Optional[Subgait]:
        """Get the next subgait from the graph.

        If the setpoints should stop it looks up the stop attribute in the gait graph.
        Otherwise it looks up the to attribute in the gait graph.
        :return:
        """
        next_subgait_name = None
        if self._should_stop:
            next_subgait_name = self.graph[
                (self._current_subgait.subgait_name, self.graph.STOP)
            ]
        if next_subgait_name is None:
            next_subgait_name = self.graph[
                (self._current_subgait.subgait_name, self.graph.TO)
            ]

        if next_subgait_name == self.graph.END:
            next_subgait = None
        else:
            next_subgait = self.subgaits[next_subgait_name]
        return next_subgait

    def transition(self, transition_request):
        """
        Request to transition between two subgaits with increasing or decreasing
        size of the step.
        :param transition_request:
        :return: whether the transition can be executed
        """

        if self._is_transitioning or self._should_stop:
            return False

        if transition_request == TransitionRequest.DECREASE_SIZE:
            name = self.graph[
                (self._current_subgait.subgait_name, self.graph.DECREASE_SIZE)
            ]
        elif transition_request == TransitionRequest.INCREASE_SIZE:
            name = self.graph[
                (self._current_subgait.subgait_name, self.graph.INCREASE_SIZE)
            ]
        else:
            return False

        if name is not None:
            self._transition_to_subgait = self.subgaits[name]
            return True
        return False

    def stop(self):
        """Called when the current gait should be stopped. Return a boolean
        for whether the stopping was succesfull."""
        if (
            self.graph.is_stoppable()
            and not self._is_transitioning
            and self._transition_to_subgait is None
        ):
            self._should_stop = True
            return True
        else:
            return False

    def end(self):
        """Called when the gait has finished."""
        self._current_subgait = None

    def set_subgait_versions(self, robot, gait_directory, version_map):
        """
        Change the versions of the subgaits.
        :param robot: The robot model used.
        :param gait_directory: The directory where the gaits are located.
        :param version_map: The map with the new versions to use.
        """
        if self._current_subgait is None:
            super(SetpointsGait, self).set_subgait_versions(
                robot, gait_directory, version_map
            )
        else:
            raise GaitError(
                "Cannot change subgait version while gait is being executed"
            )

    def _make_transition_subgait(self) -> TransitionSubgait:
        """
        Creates the transition subgait from the next subgait to the
        subgait stored in _transition_to_subgait.
        :return: The transition subgait.
        """
        old_subgait = self.subgaits[
            self.graph[(self._current_subgait.subgait_name, self.graph.TO)]
        ]
        new_subgait = self.subgaits[
            self.graph[(self._transition_to_subgait.subgait_name, self.graph.TO)]
        ]
        return TransitionSubgait.from_subgaits(
            old_subgait,
            new_subgait,
            "{s}_transition".format(s=self._transition_to_subgait.subgait_name),
        )

    def _command_from_current_subgait(self):
        return TrajectoryCommand.from_subgait(self._current_subgait, self._start_time)

    def _update_end_time(self):
        self._end_time = self._start_time + self._current_subgait.duration

    def _update_time_stamps(
        self, current_time: Time, previous_subgait: Subgait, next_subgait: Subgait
    ):
        """
        Update the starting time and end time.
        :param current_time:
        :param previous_subgait:
        :param next_subgait:
        :return:
        """
        if not self._scheduled_early:
            self._start_time = current_time
        else:
            self._start_time += previous_subgait.duration
        self._end_time = self._start_time + next_subgait.duration

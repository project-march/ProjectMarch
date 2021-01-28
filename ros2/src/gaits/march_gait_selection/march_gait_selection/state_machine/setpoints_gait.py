from march_gait_selection.dynamic_gaits.transition_subgait import TransitionSubgait
from march_utility.exceptions.gait_exceptions import GaitError
from march_utility.gait.gait import Gait
from march_utility.utilities.duration import Duration

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
        self._time_since_start = Duration(0)

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

    def start(self):
        """
        Start the gait, sets current subgait to the first subgait, resets the
        time and generates the first trajectory.
        :return: A JointTrajectory message with the trajectory of the first subgait.
        """
        self._current_subgait = self.subgaits[self.graph.start_subgaits()[0]]
        self._should_stop = False
        self._transition_to_subgait = None
        self._is_transitioning = False
        self._time_since_start = Duration(0)
        return self._current_subgait.to_joint_trajectory_msg()

    def update(self, elapsed_time: Duration):
        """
        Update the progress of the gait, should be called regularly.
        If the current subgait is still running, this does nothing.
        If the gait should be stopped, this will be done
        If the current subgait is done, it will start the next subgait
        :param elapsed_time:
        :return: trajectory, is_finished
        """
        self._time_since_start += elapsed_time
        if self._time_since_start < self._current_subgait.duration:
            return None, False

        return self._update_next_subgait()

    def _update_next_subgait(self):
        if self._should_stop:
            next_subgait = self._stop()

        elif self._transition_to_subgait is not None and not self._is_transitioning:
            return self._transition_subgait(), False

        elif self._transition_to_subgait is not None and self._is_transitioning:
            next_subgait = self._transition_to_subgait.subgait_name
            self._transition_to_subgait = None
            self._is_transitioning = False

        else:
            # If there is no transition subgait that has to be used, go to TO subgait
            next_subgait = self.graph[
                (self._current_subgait.subgait_name, self.graph.TO)
            ]

        if next_subgait == self.graph.END:
            return None, True
        self._current_subgait = self.subgaits[next_subgait]
        trajectory = self._current_subgait.to_joint_trajectory_msg()

        self._time_since_start = Duration(
            0
        )  # New subgait is started, so reset the time
        return trajectory, False

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

    def _stop(self):
        next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.STOP)]
        if next_subgait is None:
            next_subgait = self.graph[
                (self._current_subgait.subgait_name, self.graph.TO)
            ]
        else:
            self._should_stop = False
        return next_subgait

    def _transition_subgait(self):
        """
        Creates the transition subgait message from the next subgait to the
        subgait stored in _transition_to_subgait
        :return: The trajectory message for the transition step
        """
        old_subgait = self.subgaits[
            self.graph[(self._current_subgait.subgait_name, self.graph.TO)]
        ]
        new_subgait = self.subgaits[
            self.graph[(self._transition_to_subgait.subgait_name, self.graph.TO)]
        ]
        transition_subgait = TransitionSubgait.from_subgaits(
            old_subgait,
            new_subgait,
            "{s}_transition".format(s=self._transition_to_subgait.subgait_name),
        )
        self._current_subgait = transition_subgait
        self._time_since_start = Duration(0)
        self._is_transitioning = True
        return transition_subgait.to_joint_trajectory_msg(), False

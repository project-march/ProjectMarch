from copy import deepcopy
from march_gait_selection.state_machine.home_gait import HomeGait
from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_shared_classes.gait.setpoint import Setpoint


class SemiDynamicSetpointsGait(SetpointsGait):

    def __init__(self, gait_name, subgaits, graph):
        super(SemiDynamicSetpointsGait, self).__init__(f'dynamic_{gait_name}', subgaits, graph)
        self._should_freeze = False
        self._is_frozen = False
        self._freeze_duration = 0

    @property
    def can_freeze(self):
        """Returns whether the gait has the ability to freeze."""
        return True

    def freeze(self, duration: float = 3.0):
        self._should_freeze = True
        self._freeze_duration = duration
        return True

    def update(self, elapsed_time, logger):
        """
        Update the progress of the gait, should be called regularly.
        If the current subgait is still running, this does nothing.
        If the gait should be stopped, this will be done
        If the current subgait is done, it will start the next subgait
        :param elapsed_time:
        :return: trjectory, is_finished
        """
        self._time_since_start += elapsed_time
        if self._should_freeze:
            trajectory = self._execute_freeze(logger)
            self._time_since_start = 0.0
            return trajectory, False

        if self._time_since_start < self._current_subgait.duration:
            return None, False

        if self._should_stop:
            next_subgait = self._stop()

        elif self._transition_to_subgait is not None and not self._is_transitioning:
            return self._transition_subgait()

        elif self._transition_to_subgait is not None and self._is_transitioning:
            next_subgait = self._transition_to_subgait.subgait_name
            self._transition_to_subgait = None
            self._is_transitioning = False

        elif self._is_frozen:
            self._current_subgait = self._subgait_after_freeze
            trajectory = self._current_subgait.to_joint_trajectory_msg()
            self._time_since_start = 0.0 # New subgait is started, so reset the time
            self._is_frozen = False
            return trajectory, False
        else:
            # If there is transition to do, go to next (TO) subgait
            next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.TO)]

        if next_subgait == self.graph.END:
            return None, True
        self._current_subgait = self.subgaits[next_subgait]
        trajectory = self._current_subgait.to_joint_trajectory_msg()
        self._time_since_start = 0.0 # New subgait is started, so reset the time
        return trajectory, False

    def _execute_freeze(self, logger):
        if self._time_since_start < self._current_subgait.duration:
            self._subgait_after_freeze = deepcopy(self._current_subgait)
            for joint in self._subgait_after_freeze:
                joint.from_begin_point(self._time_since_start)

        freeze_subgait = deepcopy(self._current_subgait)
        freeze_subgait.duration = self._freeze_duration
        position = self._current_position(self._time_since_start)
        for joint in freeze_subgait.joints:
            joint.setpoints = [Setpoint(time=self._freeze_duration,
                                        position=position[joint.name],
                                        velocity=0)]
        self._current_subgait = freeze_subgait
        self._should_freeze = False
        self._is_frozen = True
        return freeze_subgait.to_joint_trajectory_msg()

    def _current_position(self, elapsed_time):
        return {joint.name: joint.get_interpolated_setpoint(elapsed_time).position for
                joint in self._current_subgait}
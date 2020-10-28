from march_gait_selection.march_gait_selection.state_machine.home_gait import HomeGait
from march_gait_selection.state_machine.setpoints_gait import SetpointsGait


class SemiDynamicSetpointsGait(SetpointsGait):

    def __init__(self, gait_name, subgaits, graph):
        super(SemiDynamicSetpointsGait, self).__init__(gait_name, subgaits, graph)
        self._should_freeze = False
        self._is_frozen = False
        self._freeze_duration = 0

    def freeze(self, duration: float = 3.0):
        self._should_freeze = True
        self._freeze_duration = duration
        return True

    def _current_position(self, elapsed_time):
        return [joint.get_interpolated_setpoint(elapsed_time) for
                joint in self._current_subgait]

    def update(self, elapsed_time):
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
            ## Set the rest of current subgait as next subgait
            if self._time_since_start < self._current_subgait.duration:
                self._transition_to_subgait
            ## Set a freeze subgait for the freeze duration
            self._current_subgait = HomeGait(name='freeze',
                                             position=self._current_position(self._time_since_start),
                                             gait_type='walk_like',
                                             duration=self._freeze_duration)

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

        else:
            # If there is no next subgait to transition to, go to TO subgait
            next_subgait = self.graph[(self._current_subgait.subgait_name, self.graph.TO)]

        if next_subgait == self.graph.END:
            return None, True
        self._current_subgait = self.subgaits[next_subgait]
        trajectory = self._current_subgait.to_joint_trajectory_msg()
        self._time_since_start = 0.0 # New subgait is started, so reset the time
        return trajectory, False

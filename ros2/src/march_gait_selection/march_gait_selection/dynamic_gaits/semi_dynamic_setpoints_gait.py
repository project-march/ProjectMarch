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
    def can_freeze(self) -> bool:
        """Returns whether the gait has the ability to freeze. This is meant
        for noticing the step height and ending the subgait earlier. This is
        therefore not possible during the first second of the subgait, to
        prevent accidental freezing."""
        if self._time_since_start < 1 and not self._is_frozen:
            return False
        return True

    def freeze(self, duration: float = 3.0):
        """
        If the subgait can freeze it will freeze for the given duration, this
        will later be changed to start the next subgait more dynamically
        after the short freeze
        :param duration: How long to freeze in the current position
        """
        if self.can_freeze:
            self._should_freeze = True
            self._freeze_duration = duration

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
        if self._should_freeze and self.can_freeze:
            trajectory = self._execute_freeze()
            self._time_since_start = 0.0
            return trajectory, False

        if self._time_since_start < self._current_subgait.duration:
            return None, False

        if self._is_frozen:
            self._current_subgait = self._subgait_after_freeze
            trajectory = self._current_subgait.to_joint_trajectory_msg()
            self._time_since_start = 0.0 # New subgait is started, so reset the time
            self._is_frozen = False
            return trajectory, False

        return self._update_next_subgait()

    def _execute_freeze(self):
        """
        Freezes the subgait, currently this means that there is a new subgait
        started of the given freeze duration which ends at the current position.
        If this happens in the middle of a subgait, it plans the rest of the
        original subgait after the freeze.
        :return:
        """
        self._subgait_after_freeze = self._subgait_after_freeze()
        self._current_subgait = self._freeze_subgait()
        self._should_freeze = False
        self._is_frozen = True
        return self._current_subgait.to_joint_trajectory_msg()

    def _subgait_after_freeze(self):
        """
        Generates the subgait that should be executed after the freezing.
        :return: The subgait to execute after the freeze
        """
        if self._time_since_start < self._current_subgait.duration:
            subgait_after_freeze = deepcopy(self._current_subgait)
            for joint in subgait_after_freeze:
                joint.from_begin_point(self._time_since_start)
        else:
            subgait_after_freeze = self.graph[
                (self._current_subgait.subgait_name, self.graph.TO)]
        return subgait_after_freeze

    def _freeze_subgait(self):
        """
        Generates a subgait of the freeze duration based on the current position.
        :return: A subgait to freeze in current position
        """
        freeze_subgait = deepcopy(self._current_subgait)
        freeze_subgait.duration = self._freeze_duration
        freeze_subgait.subgait_name = 'dynamic_freeze'
        position = self._position_after_time(self._time_since_start)
        for joint in freeze_subgait.joints:
            joint.setpoints = [Setpoint(time=self._freeze_duration,
                                        position=position[joint.name],
                                        velocity=0)]
        return freeze_subgait

    def _position_after_time(self, elapsed_time):
        """
        The position that the exoskeleton should be in after a certain elapsed
        time of the current subgait.
        :param elapsed_time: The time since the start of the subgait in secs.
        :return: dict with joints and joint positions
        """
        return {joint.name: joint.get_interpolated_setpoint(elapsed_time).position for
                joint in self._current_subgait}

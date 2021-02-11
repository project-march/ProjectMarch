from copy import deepcopy
from march_gait_selection.state_machine.setpoints_gait import SetpointsGait
from march_utility.gait.subgait import Subgait
from march_utility.utilities.duration import Duration

SHOULD_NOT_FREEZE_FIRST_SECS = Duration(seconds=0.3)


class SemiDynamicSetpointsGait(SetpointsGait):
    """ A semi-dynamic version of the setpoints gait, implements a freeze functionality """

    def __init__(self, gait_name, subgaits, graph):
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
        if (
            self._time_since_start < SHOULD_NOT_FREEZE_FIRST_SECS
            or self._should_freeze
            or self._is_frozen
        ):
            return False
        return True

    def freeze(self, duration: Duration = Duration(seconds=3)):
        """
        If the subgait can freeze it will freeze for the given duration, this
        will later be changed to start the next subgait more dynamically
        after the short freeze
        :param duration: How long to freeze in the current position
        """
        if self.can_freeze:
            self._should_freeze = True
            self._freeze_duration = duration

    def update(self, elapsed_time: Duration):
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
            trajectory = self._execute_freeze()
            self._time_since_start = Duration(
                0
            )  # New subgait is started, so reset the time
            return trajectory, False

        # If the current subgait is not finished, no new trajectory is necessary
        if self._time_since_start < self._current_subgait.duration:
            return None, False

        # If the subgait is finished and it was frozen, execute the subgait after freeze
        if self._is_frozen:
            self._current_subgait = self._subgait_after_freeze
            trajectory = self._current_subgait.to_joint_trajectory_msg()
            self._time_since_start = Duration(
                0
            )  # New subgait is started, so reset the time
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
        self._freeze_position = self._position_after_time(self._time_since_start)
        self._previous_subgait = self._current_subgait.subgait_name
        self._subgait_after_freeze = self.subgait_after_freeze()
        self._current_subgait = self._freeze_subgait()
        self._should_freeze = False
        self._is_frozen = True
        return self._current_subgait.to_joint_trajectory_msg()

    def subgait_after_freeze(self):
        """
        Generates the subgait that should be executed after the freezing.
        :return: The subgait to execute after the freeze
        """
        if self._time_since_start < self._current_subgait.duration:
            # The subgait after freeze is a copy of the current gait from
            # the current time point
            subgait_after_freeze = deepcopy(self._current_subgait)
            for joint in subgait_after_freeze:
                joint.from_begin_point(
                    self._time_since_start, self._freeze_position[joint.name]
                )
            subgait_after_freeze.duration -= self._time_since_start
            subgait_after_freeze.subgait_name = "freeze_subgait_after"
            # Add the subgait after the freeze to the subgait graph
            self.graph.graph[subgait_after_freeze.subgait_name] = {
                "to": self.graph[(self._previous_subgait, self.graph.TO)]
            }
        else:
            # If the current subgait was already done,
            # go to the next subgait after freeze
            subgait_after_freeze = self.subgaits[
                self.graph[(self._previous_subgait, self.graph.TO)]
            ]
        return subgait_after_freeze

    def _freeze_subgait(self):
        """
        Generates a subgait of the freeze duration based on the current position.
        :return: A subgait to freeze in current position
        """
        new_dict = {
            "description": "A subgait that stays in the same position",
            "duration": self._freeze_duration.nanoseconds,
            "gait_type": self._current_subgait.gait_type,
            "joints": dict(
                [
                    (
                        joint.name,
                        [
                            {
                                "position": self._freeze_position[joint.name],
                                "time_from_start": self._freeze_duration.nanoseconds,
                                "velocity": 0,
                            }
                        ],
                    )
                    for joint in self._current_subgait.joints
                ]
            ),
        }

        # freeze subgait
        return Subgait.from_dict(
            robot=self._current_subgait.robot,
            subgait_dict=new_dict,
            gait_name=self.gait_name,
            subgait_name="freeze",
            version="Only version, generated from code",
        )

    def _position_after_time(self, elapsed_time: Duration):
        """
        The position that the exoskeleton should be in after a certain elapsed
        time of the current subgait.
        :param elapsed_time: The time since the start of the subgait in secs.
        :return: dict with joints and joint positions
        """
        return {
            joint.name: joint.get_interpolated_setpoint(elapsed_time).position
            for joint in self._current_subgait
        }

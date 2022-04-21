"""Author: ???."""

from __future__ import annotations
from typing import Optional

from attr import dataclass
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand


@dataclass
class GaitUpdate:
    """The GaitUpdate is a data structure for the different gaits to communicate with the gait state machine."""

    new_trajectory_command: Optional[TrajectoryCommand]
    is_new_subgait: bool
    is_finished: bool

    @staticmethod
    def empty() -> GaitUpdate:
        """A GaitUpdate with no information.

        Returns:
            GaitUpdate: empty GaitUpdate for when no action should be taken
        """
        return GaitUpdate(None, False, False)

    @staticmethod
    def finished() -> GaitUpdate:
        """A GaitUpdate with the is_finished flag set to true.

        Returns:
            GaitUpdate: GaitUpdate that stops the current gait
        """
        return GaitUpdate(None, False, True)

    @staticmethod
    def subgait_updated() -> GaitUpdate:
        """A GaitUpdate with the is_new_subgait flag set to true.

        Returns:
            GaitUpdate: GaitUpdate without a new trajectory, but with is_new_subgait set to true. Use when the
                TrajectoryScheduler should not schedule a new TrajectoryCommand, but the current subgait should
                be updated.
        """
        return GaitUpdate(None, True, False)

    @staticmethod
    def should_schedule(command: TrajectoryCommand) -> GaitUpdate:
        """A GaitUpdate that contains a new TrajectoryCommand.

        Also, the is_new_subgait flag set to true indicating that the subgait has updated.

        Returns:
            GaitUpdate: Use when the TrajectoryScheduler should schedule a new TrajectoryCommand, and the current
                subgait should not be updated.
        """
        return GaitUpdate(command, True, False)

    @staticmethod
    def should_schedule_early(command: TrajectoryCommand) -> GaitUpdate:
        """A GaitUpdate that contains a new TrajectoryCommand, but the current subgait is not be updated yet.

        Returns:
            Use when the TrajectoryScheduler should schedule a new TrajectoryCommand, but the current subgait should
                not be updated.
        """
        return GaitUpdate(command, False, False)

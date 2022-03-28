from march_gait_selection.state_machine.state_machine_input import TransitionRequest
from march_utility.utilities.duration import Duration
from march_utility.gait.edge_position import EdgePosition
from rclpy.time import Time

from .gait_update import GaitUpdate


class GaitInterface:
    """The interface that defines the properties and functions that every gait
    should have."""

    @property
    def name(self) -> str:
        """Returns the name of the gait."""
        return ""

    @property
    def subgait_name(self) -> str:
        """Returns the name of the currently executing trajectory."""
        return ""

    @property
    def version(self) -> str:
        """Returns the version of the currently executing trajectory."""
        return ""

    @property
    def duration(self) -> Duration:
        """Returns the duration in seconds of the currently executing trajectory
        from the start of the gait."""
        return Duration(0)

    @property
    def gait_type(self) -> str:
        """Returns a gait type of the currently executing trajectory."""
        return ""

    @property
    def starting_position(self) -> EdgePosition:
        """Returns the starting position of all joints."""
        return

    @property
    def final_position(self) -> EdgePosition:
        """Returns the position of all the joints after the gait has ended."""
        return

    @property
    def subsequent_subgaits_can_be_scheduled_early(self) -> bool:
        """Return whether this gait can be scheduled early, default is False."""
        return False

    @property
    def first_subgait_can_be_scheduled_early(self) -> bool:
        """Return whether this gait can be started early, this means that the first
        subgait will be delayed, with the first_subgait_delay, default is False."""
        return False

    def start(self, current_time: Time) -> GaitUpdate:
        """Start the gait.

        Returns:
             GaitUpdate: a GaitUpdate that usually contains a TrajectoryCommand."""
        return GaitUpdate.empty()

    def update(self, current_time: Time) -> GaitUpdate:
        """Give an update on the progress of the gait.

        Args:
           current_time (Time): Current time
        Returns:
             GaitUpdate: a GaitUpdate that may contain a TrajectoryCommand, and any of the
                flags set to true, depending on the state of the Gait.
        """
        return GaitUpdate.finished()

    def transition(self, transition_request: TransitionRequest) -> bool:
        """Requests a special transition.

        Args:
             transition_request: (TransitionRequest): request on what special transition to perform
        Returns:
            bool: True when the request has been accepted, False otherwise.
        """
        return False

    def stop(self) -> bool:
        """Called when the gait has been instructed to stop.

        Returns:
            bool: True when the stop action has been accepted, False otherwise.
        """
        return False

    def end(self) -> bool:
        """Called when the gait has finished."""

from typing import Optional

from march_gait_selection.state_machine.state_machine_input import TransitionRequest
from march_utility.gait.joint_trajectory import JointTrajectory
from march_utility.utilities.duration import Duration


class GaitInterface(object):
    """The interface that defines the properties and functions that every gait
    should have."""

    @property
    def name(self) -> str:
        """Returns the name of the gait."""
        return ""

    @property
    def can_freeze(self) -> bool:
        """Returns whether the gait has a freeze functionality."""
        return False

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
    def starting_position(self) -> dict:
        """Returns the starting position of all joints."""
        return {}

    @property
    def final_position(self) -> dict:
        """Returns the position of all the joints after the gait has ended."""
        return {}

    def start(self) -> Optional[JointTrajectory]:
        """Called when the gait has been selected for execution and returns an
        optional starting trajectory."""
        return None

    def update(self, elapsed_time: Duration) -> (JointTrajectory, bool):
        """Called in a loop with the elapsed time since the last update.

        :param float elapsed_time: Elapsed time in seconds since the last update

        :returns A pair of a trajectory and a flag. The trajectory that will be
                 set as the new goal for the controller, can be None. The flag
                 indicates whether the gait has finished.
        """
        return None, True

    def transition(self, transition_request: TransitionRequest) -> bool:
        """Requests a special transition.

        :param TransitionRequest transition_request: request on what special
               transition to perform

        :returns True when the request has been accepted, False otherwise.
        """
        return False

    def stop(self) -> bool:
        """Called when the gait has been instructed to stop.

        :returns True when the stop action has been accepted, False otherwise.
        """
        return False

    def end(self) -> bool:
        """Called when the gait has finished."""
        pass

"""Author: Marten Haitjema, MVII"""

from rclpy.node import Node
from march_gait_selection.dynamic_interpolation.dynamic_setpoint_gait import (
    DynamicSetpointGait,

)
from march_gait_selection.state_machine.trajectory_scheduler import TrajectoryCommand
from march_utility.utilities.duration import Duration

from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import Point

DEFAULT_FIRST_SUBGAIT_START_DELAY = Duration(0)


class DynamicSetpointGaitClose(DynamicSetpointGait):
    """Close gait based on dynamic setpoint gait. Should be used
    when a previous step could not be executed and the gait needs
    to be closed.

    :param gait_selection_node: the gait selection node
    :type gait_selection_node: Node
    """

    def __init__(self, gait_selection_node: Node):
        super().__init__(gait_selection_node)
        self.gait_name = "dynamic_walk_close"

    def _get_trajectory_command(self, start=False, stop=True) -> TrajectoryCommand:
        """Return a TrajectoryCommand based on current subgait_id.

        :param start: whether it is a start gait or not
        :type start: bool
        :param stop: whether it is a stop gait or not
        :type stop: bool

        :return: TrajectoryCommand with the current subgait and start time.
        :rtype: TrajectoryCommand
        """
        if self._start_is_delayed:
            self._end_time = self._start_time

        self.foot_location = self._get_foot_location(self.subgait_id)

        return self._get_first_feasible_trajectory(start, stop)

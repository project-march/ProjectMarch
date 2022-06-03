"""Author: Marten Haitjema, MVII."""

from march_utility.utilities.logger import Logger
from march_gait_selection.dynamic_interpolation.trajectory_command_factories.trajectory_command_factory import (
    TrajectoryCommandFactory,
)

from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition


class TrajectoryCommandFactoryClose(TrajectoryCommandFactory):
    """TrajectoryCommandFactory class but with a hard coded foot location for the close gait."""

    def __init__(self, gait, points_handler):
        super().__init__(gait, points_handler)
        self._gait = gait
        self._points_handler = points_handler
        self._logger = Logger(self._gait.gait_selection, __class__.__name__)
        self._trajectory_failed = False
        # TODO: remove hardcoded foot location after bug in stop gait is fixed.
        self.foot_location = FootPosition(duration=1.5, processed_point=Point(x=0.5, y=0.03, z=0.446))

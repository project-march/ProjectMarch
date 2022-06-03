"""Author: Marten Haitjema, MVII."""

from typing import Optional

from march_gait_selection.dynamic_interpolation.camera_point_handlers.camera_points_handler import CameraPointsHandler
from march_utility.utilities.logger import Logger
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_shared_msgs.msg import FootPosition


class SimulatedPointsHandler(CameraPointsHandler):
    """Class to handle communication between gaits and simulated foot locations."""

    def __init__(self, gait):
        self._gait = gait
        self._logger = Logger(self._gait.node, __class__.__name__)
        super().__init__(gait)

    def _create_subscribers(self) -> None:
        """Create subscribers to listen to simulated points."""
        self._gait.node.create_subscription(
            FootPosition,
            "/march/fixed_foot_position",
            self._update_foot_location,
            DEFAULT_HISTORY_DEPTH,
        )

    def _update_foot_location(self, foot_position: FootPosition) -> None:
        """Update left and right foot position."""
        self._fixed_foot_location = foot_position

    def get_foot_location(self, subgait_id: str) -> Optional[FootPosition]:
        """Return the fixed foot location.

        Returns:
            FootPosition: either the left or right foot position or none
        """
        return self._fixed_foot_location

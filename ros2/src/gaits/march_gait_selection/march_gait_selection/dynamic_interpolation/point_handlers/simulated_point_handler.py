"""Author: Marten Haitjema, MVII."""

from typing import Optional

from march_gait_selection.dynamic_interpolation.point_handlers.point_handler import PointHandler
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_shared_msgs.msg import FootPosition


class SimulatedPointHandler(PointHandler):
    """Class to handle communication between gaits and simulated foot locations."""

    def __init__(self, node):
        super().__init__(node)

    def _create_subscribers(self) -> None:
        """Create subscribers to listen to simulated points."""
        self._node.create_subscription(
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

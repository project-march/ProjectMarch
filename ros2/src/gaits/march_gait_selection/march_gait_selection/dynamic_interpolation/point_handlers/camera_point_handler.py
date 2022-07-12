"""Author: Marten Haitjema, MVII."""

from typing import Optional
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
from march_gait_selection.dynamic_interpolation.point_handlers.point_handler import PointHandler


class CameraPointHandler(PointHandler):
    """Class to handle all communications between CoViD and gaits.

    Args:
        node: the gait node
    """

    def __init__(self, node):
        super().__init__(node)

    def _create_subscribers(self) -> None:
        """Create subscribers to listen to points given by depth cameras."""
        self._node.create_subscription(
            FootPosition,
            "/march/processed_foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )
        self._node.create_subscription(
            FootPosition,
            "/march/processed_foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )

    def get_foot_location(self, subgait_id: str) -> Optional[FootPosition]:
        """Returns the right or left foot position based upon the subgait_id.

        Args:
            subgait_id (str): Either right_swing or left_swing
        Returns:
            FootPosition: either the left or right foot position or none
        """
        if subgait_id == "left_swing":
            return self._foot_location_left
        elif subgait_id == "right_swing":
            return self._foot_location_right
        else:
            return None

"""Author: Marten Haitjema, MVII."""

from abc import ABC, abstractmethod
from typing import Optional, Tuple
from rclpy.time import Time

from march_gait_selection.dynamic_interpolation.gaits.dynamic_joint_trajectory import NANOSECONDS_TO_SECONDS
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

FOOT_LOCATION_TIME_OUT = Duration(0.5)


class PointHandler(ABC):
    """Class to handle all communications between CoViD and gaits.

    Args:
        node: the gait node

    Attributes:
        pub_right (rclpy.Publisher): Publisher for right chosen foot position
        pub_left (rclpy.Publisher): Publisher for left chosen foot position

        _foot_location_right (FootPosition): the latest right foot position
        _foot_location_left (FootPosition): the latest left foot position
        _node: the gait node
        _logger: used to log with class name as message prefix
    """

    def __init__(self, node):
        self._node = node
        self._logger = self._node.get_logger().get_child(__class__.__name__)
        self._create_subscribers()
        self._create_publishers()

    @abstractmethod
    def _create_subscribers(self) -> None:
        """Create subscribers that listen to published foot locations."""

    @abstractmethod
    def get_foot_location(self, subgait_id: str) -> Optional[FootPosition]:
        """Returns the foot location.

        Returns:
            FootPosition: either the left or right foot position or none
        """

    def _create_publishers(self) -> None:
        """Create publishers to publish chosen point back to covid."""
        self.pub_right = self._node.create_publisher(
            FootPosition,
            "/march/chosen_foot_position/right",
            DEFAULT_HISTORY_DEPTH,
        )
        self.pub_left = self._node.create_publisher(
            FootPosition,
            "/march/chosen_foot_position/left",
            DEFAULT_HISTORY_DEPTH,
        )

    def _callback_right(self, foot_location: FootPosition) -> None:
        """Update the right foot position with the latest point published on the CoViD-topic.

        Args:
            foot_location (FootPosition): a Point containing the x, y, and z location
        """
        self._foot_location_right = foot_location

    def _callback_left(self, foot_location: FootPosition) -> None:
        """Update the left foot position with the latest point published on the CoViD-topic.

        Args:
            foot_location (FootPosition): a Point containing the x, y, and z location
        """
        self._foot_location_left = foot_location

    def publish_chosen_foot_position(self, subgait_id: str, foot_position: FootPosition) -> None:
        """Publish the point to which the step is planned.

        Args:
            subgait_id (str): whether it is a right or left swing
            foot_position (FootPosition): point message to which step is planned
        """
        if subgait_id == "left_swing":
            self.pub_left.publish(foot_position)
        elif subgait_id == "right_swing":
            self.pub_right.publish(foot_position)

    def is_foot_location_too_old(self, foot_location: FootPosition) -> Tuple[bool, str]:
        """Checks if the foot_location given by CoViD is not older than FOOT_LOCATION_TIME_OUT.

        Args:
            foot_location (FootPosition): FootPosition message that should be checked
        Returns:
            bool: True if message is not more than 0.5 seconds old, else False
        """
        msg_time = Time(
            seconds=foot_location.header.stamp.sec,
            nanoseconds=foot_location.header.stamp.nanosec,
        )
        current_time = Time(
            seconds=self._node.get_clock().now().seconds_nanoseconds()[0],
            nanoseconds=self._node.get_clock().now().seconds_nanoseconds()[1],
        )
        time_difference = current_time - msg_time
        readable_time_difference = f"{time_difference.nanoseconds / NANOSECONDS_TO_SECONDS}"
        self._logger.debug(
            f"Time difference between CoViD foot location and current time: {readable_time_difference}.",
        )

        if time_difference > FOOT_LOCATION_TIME_OUT:
            msg = (
                f"Foot location is more than {FOOT_LOCATION_TIME_OUT} seconds old, time difference is "
                f"{readable_time_difference} seconds. Stopping gait."
            )
            return True, msg

        return False, ""

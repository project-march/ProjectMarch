"""Author: Marten Haitjema, MVII."""

from typing import Optional
from rclpy.time import Time

from march_gait_selection.dynamic_interpolation.gaits.dynamic_joint_trajectory import NANOSECONDS_TO_SECONDS
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.duration import Duration
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

FOOT_LOCATION_TIME_OUT = Duration(0.5)


class CameraPointsHandler:
    """Class to handle all communications between CoViD and gaits.

    Args:
        gait: the gait class

    Attributes:
        pub_right (rclpy.Publisher): Publisher for right chosen foot position
        pub_left (rclpy.Publisher): Publisher for left chosen foot position

        _foot_location_right (FootPosition): the latest right foot position
        _foot_location_left (FootPosition): the latest left foot position
        _gait: the gait class
        _logger: used to log with class name as message prefix
    """

    def __init__(self, gait):
        self._gait = gait
        self._logger = gait.gait_selection.get_logger().get_child(__class__.__name__)
        self._create_subscribers()
        self._create_publishers()

    def _create_subscribers(self) -> None:
        self._gait.gait_selection.create_subscription(
            FootPosition,
            "/march/processed_foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )
        self._gait.gait_selection.create_subscription(
            FootPosition,
            "/march/processed_foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )

    def _create_publishers(self) -> None:
        self.pub_right = self._gait.gait_selection.create_publisher(
            FootPosition,
            "/march/chosen_foot_position/right",
            DEFAULT_HISTORY_DEPTH,
        )
        self.pub_left = self._gait.gait_selection.create_publisher(
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

    def is_foot_location_too_old(self, foot_location: FootPosition) -> bool:
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
            seconds=self._gait.gait_selection.get_clock().now().seconds_nanoseconds()[0],
            nanoseconds=self._gait.gait_selection.get_clock().now().seconds_nanoseconds()[1],
        )
        time_difference = current_time - msg_time
        readable_time_difference = f"{time_difference.nanoseconds / NANOSECONDS_TO_SECONDS}"
        self._logger.debug(
            f"Time difference between CoViD foot location and current time: {readable_time_difference}.",
        )

        if time_difference > FOOT_LOCATION_TIME_OUT:
            self._logger.warn(
                f"Foot location is more than {FOOT_LOCATION_TIME_OUT} seconds old, time difference is "
                f"{readable_time_difference} seconds. Stopping gait."
            )
            self._gait._end = True
            return True

        return False

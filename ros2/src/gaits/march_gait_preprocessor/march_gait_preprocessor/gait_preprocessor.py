"""Author: Marten Haitjema, MVII"""

from rclpy.node import Node
from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

NODE_NAME = "gait_preprocessor_node"
DURATION_SCALING_FACTOR = 5
# Offsets are used to account for the difference in points between
# covid (middle of foot) and gait (at the heel)
X_OFFSET = 0.1
Y_OFFSET = 0.05


class GaitPreprocessor(Node):
    """Node for processing the points coming from the vision package,
    before sending them to the gait generation. This node can also be
    used to simulate fake points."""

    def __init__(self):
        super().__init__(
            NODE_NAME, automatically_declare_parameters_from_overrides=True
        )

        self._set_parameters()

        self.set_simulate_points_parameter()
        self._create_publishers()

    def _set_parameters(self) -> None:
        """Read node parameters from parameter server."""
        self._simulate_points = (
            self.get_parameter("simulate_points").get_parameter_value().bool_value
        )
        self._duration = (
            self.get_parameter("duration").get_parameter_value().double_value
        )
        self._location_x = (
            self.get_parameter("location_x").get_parameter_value().double_value
        )
        self._location_y = (
            self.get_parameter("location_y").get_parameter_value().double_value
        )

    def _create_covid_subscribers(self) -> None:
        """Create subscribers to the topics on which covid
        publishes found points"""
        self.subcription_left = self.create_subscription(
            FootPosition,
            "/foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )
        self.subcription_right = self.create_subscription(
            FootPosition,
            "/foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )

    def _create_publishers(self) -> None:
        """Create publishers for the topics gait listens to."""
        self.publisher_right = self.create_publisher(
            FootPosition,
            "/processed_foot_position/right",
            DEFAULT_HISTORY_DEPTH,
        )
        self.publisher_left = self.create_publisher(
            FootPosition,
            "/processed_foot_position/left",
            DEFAULT_HISTORY_DEPTH,
        )

    def _callback_left(self, foot_location: FootPosition) -> None:
        """Callback for new left point from covid. Makes the point
        usable for the gait."""
        foot_location_msg = self._process_foot_location(foot_location)
        self.publisher_left.publish(foot_location_msg)

    def _callback_right(self, foot_location: FootPosition) -> None:
        """Callback for new right point from covid. Makes the point
        usable for the gait."""
        foot_location_msg = self._process_foot_location(foot_location)
        self.publisher_right.publish(foot_location_msg)

    def _process_foot_location(self, foot_location: FootPosition) -> FootPosition:
        """Reformat the foot location so that gait can use it."""
        transformed_foot_location = self._get_foot_location_in_gait_axes(foot_location)
        scaled_duration = self._get_duration_scaled_to_height(
            self._duration, transformed_foot_location.y
        )

        return FootPosition(
            header=foot_location.header,
            point=transformed_foot_location,
            track_points=foot_location.track_points,
            duration=scaled_duration,
        )

    def _get_foot_location_in_gait_axes(self, foot_location: FootPosition) -> Point:
        """Transforms the point found by covid from the covid axes to the gait axes."""
        temp_y = foot_location.point.y
        point = Point()

        point.x = -foot_location.point.x + X_OFFSET
        point.y = foot_location.point.z + Y_OFFSET
        point.z = temp_y

        return point

    def _get_duration_scaled_to_height(
        self, duration: float, step_height: float
    ) -> float:
        """Scales the duration based on the absolute step height"""
        return duration + DURATION_SCALING_FACTOR * abs(step_height)

    def _publish_simulated_locations(self) -> None:
        """Publishes simulated foot locations"""
        point_msg = FootPosition()
        point_msg.header.stamp = self.get_clock().now().to_msg()

        point_msg.point.x = self._location_x
        point_msg.point.y = self._location_y
        point_msg.point.z = 0.0
        point_msg.duration = self._get_duration_scaled_to_height(
            self._duration, self._location_y
        )

        self.publisher_right.publish(point_msg)
        self.publisher_left.publish(point_msg)

    def set_simulate_points_parameter(self) -> None:
        """Creates either a publisher that publishes simulated points or a
        subsctiption on the covid points topics."""
        if self._simulate_points:
            self.destroy_subscription(self.subcription_right)
            self.destroy_subscription(self.subcription_left)
            self.timer = self.create_timer(0.1, self._publish_simulated_locations)
        else:
            self.destroy_timer(self.timer)
            self._create_covid_subscribers()

"""Author: Marten Haitjema, MVII."""

from rclpy.node import Node
from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH
import numpy as np

NODE_NAME = "gait_preprocessor_node"
DURATION_SCALING_FACTOR = 5
# Offsets are used to account for the difference in points between
# covid (middle of foot) and gait (at the heel)
X_OFFSET = 0
Y_OFFSET = -0.01
Z_OFFSET = 0.22


class GaitPreprocessor(Node):
    """Node for processing the points coming from the vision package, before sending them to the gait generation.

    This node can also be used to simulate fake points.
    """

    def __init__(self):
        super().__init__(NODE_NAME, automatically_declare_parameters_from_overrides=True)

        self.timer = None
        self.subscription_left = None
        self.subscription_right = None

        self._init_parameters()
        self.set_simulate_points_parameter()
        self._create_publishers()

    def _init_parameters(self) -> None:
        """Read node parameters from parameter server."""
        self._simulate_points = self.get_parameter("simulate_points").get_parameter_value().bool_value
        self._duration = self.get_parameter("duration").get_parameter_value().double_value
        self._location_x = self.get_parameter("location_x").get_parameter_value().double_value
        self._location_y = self.get_parameter("location_y").get_parameter_value().double_value
        self._location_z = self.get_parameter("location_z").get_parameter_value().double_value

    def _create_covid_subscribers(self) -> None:
        """Create subscribers to the topics on which covid publishes found points."""
        self.subscription_left = self.create_subscription(
            FootPosition,
            "/march/foot_position/left",
            self._callback_left,
            DEFAULT_HISTORY_DEPTH,
        )
        self.subscription_right = self.create_subscription(
            FootPosition,
            "/march/foot_position/right",
            self._callback_right,
            DEFAULT_HISTORY_DEPTH,
        )

    def _create_publishers(self) -> None:
        """Create publishers for the topics gait listens to."""
        self.publisher_right = self.create_publisher(
            FootPosition,
            "/march/processed_foot_position/right",
            DEFAULT_HISTORY_DEPTH,
        )
        self.publisher_left = self.create_publisher(
            FootPosition,
            "/march/processed_foot_position/left",
            DEFAULT_HISTORY_DEPTH,
        )

    def _callback_left(self, foot_location: FootPosition) -> None:
        """Callback for new left point from covid. Makes the point usable for the gait.

        Args:
            foot_location (FootPosition): Location given by CoViD (Computer Vision).
        """
        foot_location_msg = self._process_foot_location(foot_location)
        self.publisher_left.publish(foot_location_msg)

    def _callback_right(self, foot_location: FootPosition) -> None:
        """Callback for new right point from covid. Makes the point usable for the gait.

        Args:
            foot_location (FootPosition): Location given by CoViD (Computer Vision).
        """
        foot_location_msg = self._process_foot_location(foot_location)
        self.publisher_right.publish(foot_location_msg)

    def _process_foot_location(self, foot_location: FootPosition) -> FootPosition:
        """Reformat the foot location so that gait can use it.

        Args:
            foot_location (FootPosition): Location given by CoViD (Computer Vision).

        Returns:
            Point: Location with transformed axes and scaled duration.
        """
        transformed_foot_location = self._get_foot_location_in_gait_axes(foot_location)
        scaled_duration = self._get_duration_scaled_to_height(self._duration, transformed_foot_location.y)

        return FootPosition(
            header=foot_location.header,
            processed_point=transformed_foot_location,
            point=foot_location.point,
            point_world=foot_location.point_world,
            displacement=foot_location.displacement,
            track_points=foot_location.track_points,
            duration=scaled_duration,
        )

    @staticmethod
    def _get_foot_location_in_gait_axes(foot_location: FootPosition) -> Point:
        """Transforms the point found by covid from the covid axes to the gait axes.

        Args:
            foot_location (FootPosition): Location given by covid.

        Returns:
            Point: Foot location transformed to ik solver axes.
        """
        temp_y = foot_location.displacement.y
        point = Point()

        point.x = -foot_location.displacement.x + X_OFFSET
        point.y = foot_location.displacement.z + Y_OFFSET
        point.z = temp_y + np.sign(temp_y) * Z_OFFSET

        return point

    @staticmethod
    def _get_duration_scaled_to_height(duration: float, step_height: float) -> float:
        """Scales the duration based on the absolute step height.

        Args:
            duration (float): Duration of the step in seconds.
            step_height (float): Y-coordinate of the covid point.

        Returns:
            float: Scaled duration in seconds.
        """
        return duration + DURATION_SCALING_FACTOR * abs(step_height)

    def _publish_simulated_locations(self) -> None:
        """Publishes simulated foot locations."""
        point_msg = FootPosition()
        point_msg.header.stamp = self.get_clock().now().to_msg()

        point_msg.processed_point.x = self._location_x
        point_msg.processed_point.y = self._location_y
        point_msg.processed_point.z = self._location_z
        point_msg.duration = self._get_duration_scaled_to_height(self._duration, self._location_y)

        self.publisher_right.publish(point_msg)
        self.publisher_left.publish(point_msg)

    def set_simulate_points_parameter(self) -> None:
        """Creates either a publisher that publishes simulated points or a subscription on the covid points topics."""
        if self._simulate_points:
            self.destroy_subscription(self.subscription_right)
            self.destroy_subscription(self.subscription_left)
            self.timer = self.create_timer(0.1, self._publish_simulated_locations)
        else:
            self.destroy_timer(self.timer)
            self._create_covid_subscribers()

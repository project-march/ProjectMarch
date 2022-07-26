"""Author: Marten Haitjema, MVII."""

from rclpy.node import Node
from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition
from march_utility.utilities.node_utils import DEFAULT_HISTORY_DEPTH

NODE_NAME = "gait_preprocessor_node"
DURATION_SCALING_FACTOR = 5
# Offsets are used to account for the difference in points between
# covid (middle of foot) and gait (at the heel)
X_OFFSET = 0
Y_OFFSET = 0.020
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
        self._step_height_previous = 0.0

        self._init_parameters()
        self._create_subscribers()
        self._create_publishers()
        self.timer = self.create_timer(0.2, self._publish_simulated_locations)

    def _init_parameters(self) -> None:
        """Read node parameters from parameter server."""
        self._duration = self.get_parameter("duration").get_parameter_value().double_value
        self._location_x = self.get_parameter("location_x").get_parameter_value().double_value
        self._location_y = self.get_parameter("location_y").get_parameter_value().double_value
        self._location_z = self.get_parameter("location_z").get_parameter_value().double_value

    def _create_subscribers(self) -> None:
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
        self.create_subscription(
            FootPosition,
            "/march/chosen_foot_position/right",
            self._update_step_height_previous,
            DEFAULT_HISTORY_DEPTH,
        )
        self.create_subscription(
            FootPosition,
            "/march/chosen_foot_position/left",
            self._update_step_height_previous,
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
        self.publisher_fixed_distance = self.create_publisher(
            FootPosition,
            "/march/fixed_foot_position",
            DEFAULT_HISTORY_DEPTH,
        )

    def _callback_left(self, foot_position: FootPosition) -> None:
        """Callback for new left point from covid. Makes the point usable for the gait.

        Args:
            foot_position (FootPosition): Location given by CoViD (Computer Vision).
        """
        foot_position_msg = self._process_foot_position(foot_position)
        if self._validate_point(foot_position_msg):
            self.publisher_left.publish(foot_position_msg)

    def _callback_right(self, foot_position: FootPosition) -> None:
        """Callback for new right point from covid. Makes the point usable for the gait.

        Args:
            foot_position (FootPosition): Location given by CoViD (Computer Vision).
        """
        foot_position_msg = self._process_foot_position(foot_position)
        if self._validate_point(foot_position_msg):
            self.publisher_right.publish(foot_position_msg)

    def _update_step_height_previous(self, foot_position: FootPosition) -> None:
        """Update the _step_height_previous attribute with the height of the last chosen foot position."""
        self._step_height_previous = foot_position.processed_point.y

    def _process_foot_position(self, foot_position: FootPosition) -> FootPosition:
        """Reformat the foot location so that gait can use it.

        Args:
            foot_position (FootPosition): Location given by CoViD (Computer Vision).

        Returns:
            Point: Location with transformed axes and scaled duration.
        """
        transformed_foot_position = self._get_foot_position_in_gait_axes(foot_position)
        scaled_duration = self._get_duration_scaled_to_height(self._duration, transformed_foot_position.y)

        return FootPosition(
            header=foot_position.header,
            processed_point=transformed_foot_position,
            point=foot_position.point,
            point_world=foot_position.point_world,
            displacement=foot_position.displacement,
            track_points=foot_position.track_points,
            duration=scaled_duration,
        )

    @staticmethod
    def _get_foot_position_in_gait_axes(foot_position: FootPosition) -> Point:
        """Transforms the point found by covid from the covid axes to the gait axes.

        Args:
            foot_position (FootPosition): Location given by covid.

        Returns:
            Point: Foot location transformed to ik solver axes.
        """
        point = Point()

        point.x = -foot_position.displacement.x + X_OFFSET
        point.y = foot_position.displacement.z + Y_OFFSET
        point.z = 0.52

        return point

    def _get_duration_scaled_to_height(self, duration: float, step_height_current: float) -> float:
        """Scales the duration based on the maximum absolute step height of previous or current step.

        Args:
            duration (float): Duration of the step in seconds.
            step_height_current (float): Y-coordinate of the covid point.

        Returns:
            float: Scaled duration in seconds.
        """
        return duration + DURATION_SCALING_FACTOR * max(abs(step_height_current), abs(self._step_height_previous))

    def _publish_simulated_locations(self) -> None:
        """Publishes simulated foot locations."""
        point_msg = FootPosition()
        point_msg.header.stamp = self.get_clock().now().to_msg()

        point_msg.processed_point.x = self._location_x
        point_msg.processed_point.y = self._location_y
        point_msg.processed_point.z = self._location_z
        point_msg.duration = self._get_duration_scaled_to_height(self._duration, self._location_y)

        self.publisher_fixed_distance.publish(point_msg)

    def _validate_point(self, point: FootPosition) -> None:
        """Validates if the point sent by covid if valid."""
        return 0.15 < abs(point.processed_point.x) < 0.7 and abs(point.processed_point.y) < 0.25

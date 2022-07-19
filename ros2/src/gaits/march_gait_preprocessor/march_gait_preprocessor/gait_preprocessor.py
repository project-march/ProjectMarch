"""Author: Marten Haitjema, MVII."""

from typing import List, Tuple
from rclpy.node import Node
from geometry_msgs.msg import Point
from march_shared_msgs.msg import FootPosition, CurrentGait
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
        self._logger = self.get_logger().get_child(__class__.__name__)
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
        self._deviation_coefficient = self.get_parameter("deviation_coefficient").get_parameter_value().double_value
        self._midpoint_increase = self.get_parameter("midpoint_increase").get_parameter_value().double_value
        self._minimum_high_point_ratio = (
            self.get_parameter("minimum_high_point_ratio").get_parameter_value().double_value
        )
        self._max_deviation = self.get_parameter("max_deviation").get_parameter_value().double_value

        # Temporary parameter to test difference between old and new midpoint method
        self._new_midpoint_method = self.get_parameter("new_midpoint_method").get_parameter_value().bool_value

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
        self.create_subscription(
            CurrentGait,
            "/march/gait_selection/current_gait",
            self._reset_previous_step_height_after_close,
            1,
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
        self.publisher_left.publish(foot_position_msg)

    def _callback_right(self, foot_position: FootPosition) -> None:
        """Callback for new right point from covid. Makes the point usable for the gait.

        Args:
            foot_position (FootPosition): Location given by CoViD (Computer Vision).
        """
        foot_position_msg = self._process_foot_position(foot_position)
        self.publisher_right.publish(foot_position_msg)

    def _update_step_height_previous(self, foot_position: FootPosition) -> None:
        """Update the _step_height_previous attribute with the height of the last chosen foot position."""
        self._step_height_previous = foot_position.processed_point.y

    def _reset_previous_step_height_after_close(self, current_gait: CurrentGait) -> None:
        """Resets the _step_height_previous attribute after a close gait."""
        if current_gait.subgait in ["left_close", "right_close"]:
            self._step_height_previous = 0.0

    def _process_foot_position(self, foot_position: FootPosition) -> FootPosition:
        """Reformat the foot location so that gait can use it.

        Args:
            foot_position (FootPosition): Location given by CoViD (Computer Vision).

        Returns:
            Point: Location with transformed axes and scaled duration.
        """
        transformed_foot_position = self._transform_point_to_gait_axes(foot_position.displacement)
        if self._new_midpoint_method:
            midpoint_deviation, relative_midpoint_height = self._compute_midpoint_locations(
                foot_position.track_points, transformed_foot_position
            )
        else:
            midpoint_deviation = 0.0
            relative_midpoint_height = 0.15

        scaled_duration = self._get_duration_scaled_to_height(self._duration, transformed_foot_position.y)

        return FootPosition(
            header=foot_position.header,
            processed_point=transformed_foot_position,
            point=foot_position.point,
            point_world=foot_position.point_world,
            displacement=foot_position.displacement,
            track_points=foot_position.track_points,
            duration=scaled_duration,
            midpoint_deviation=midpoint_deviation,
            relative_midpoint_height=relative_midpoint_height,
        )

    def _compute_midpoint_locations(self, track_points: List[Point], final_point: Point) -> Tuple[float, float]:
        """Determines whether multiple midpoints are necessary, and calculates them.

        Computes their deviations from a middle fraction and relative heights to the final point.

        Args:
            track_points (List[Point]): A list of track points from the start and end point of a foot.
            final_point (Point): The final point to step to.

        Returns:
            Tuple[float, float]: A tuple containing the deviation and the relative height of the midpoints.
        """
        max_height = final_point.y

        if len(track_points) != 0:
            track_points_transformed_heights = np.asarray(
                [self._transform_point_to_gait_axes(point).y for point in track_points]
            )
            max_height = max(track_points_transformed_heights)
            high_points_ratio = (track_points_transformed_heights > (max_height - 0.025)).sum() / len(track_points)
        else:
            high_points_ratio = max_height

        if max_height <= 0.1:
            return 0.0, 0.15

        relative_midpoint_height = 0.15
        if 0.17 < max_height < 0.22:
            relative_midpoint_height = 0.15 - (max_height - 0.17)
        elif max_height > 0.22:
            relative_midpoint_height = 0.1

        absolute_midpoint_height = max(final_point.y + relative_midpoint_height, max_height + relative_midpoint_height)
        high_points_ratio = 0.0 if high_points_ratio < self._minimum_high_point_ratio else high_points_ratio
        midpoint_deviation = min(high_points_ratio * self._deviation_coefficient, self._max_deviation)

        return midpoint_deviation, absolute_midpoint_height - final_point.y

    def _transform_point_to_gait_axes(self, point: Point) -> Point:
        """Transforms the point found by covid from the covid axes to the gait axes.

        Args:
            point (Point): Location given by covid.

        Returns:
            Point: Foot location transformed to ik solver axes.
        """
        temp_y = point.y
        transformed = Point()

        transformed.x = -point.x + X_OFFSET
        transformed.y = point.z + Y_OFFSET
        transformed.z = temp_y + np.sign(temp_y) * Z_OFFSET

        return transformed

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

        if self._new_midpoint_method:
            midpoint_deviation, relative_midpoint_height = self._compute_midpoint_locations(
                [],
                point_msg.processed_point,
            )
        else:
            midpoint_deviation = 0.0
            relative_midpoint_height = 0.15

        point_msg.midpoint_deviation = midpoint_deviation
        point_msg.relative_midpoint_height = relative_midpoint_height

        self.publisher_fixed_distance.publish(point_msg)

"""Author: Tuhin Das, MVII."""

import rospy
import numpy as np
import pyrealsense2 as rs
from .utilities import convert_depth_frame_to_pointcloud, publish_point, to_point_stamped, publish_point_marker
import cv2
from typing import List, Optional, Tuple
from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import PointStamped
import tf
import tf2_ros as tf2
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2


class StoneFinder:
    """Class that looks for gray ellipses in a color frame, and return the center of the closest ellipse as a depth point.

    Args:
        left_or_right (str): Whether the class is used to find left or right points.

    Attributes:
        _dimensions (Tuple[int, int]): Dimensions of the realsense frames.
        _lower_HSV_plate (np.ndarray): Lower HSV values of the base plate color for color segmentation.
        _upper_HSV_plate (np.ndarray): Upper HSV values of the base plate color for color segmentation.
        _lower_HSV_stone (np.ndarray): Lower HSV values of the stone color for color segmentation.
        _upper_HSV_stone (np.ndarray): Upper HSV values of the stone color for color segmentation.
        _decimation_filter (rs.decimation_filter): Realsense decimation filter.
        _spatial_filter (rs.spatial_filter): Realsense spatial filter.
        _align (rs.align): Realsense alignment object.
        _last_time_point_found (Time): Timestamp of the last found point.
        _not_found_counter (int): How many iterations of 5 seconds no points were found.
        _left_or_right (string): Which leg points are found for.
        _other_frame_id (str): ROS TF frame of the standing leg.
        _camera_frame_id (str): ROS TF frame of the current realsense camera.
        _listener (TransformListener): A ROS TF frame transformation listener.
        _point_publisher (Publisher): Publisher for found points to step towards.
        _marker_publisher (Publisher): Marker publisher for the found points.

    Reconfigurable Parameters:
        _ellipse_similarity_threshold (float): Required similarity cost for a shape to be considered an ellipse.
        _minimum_ellipse_size (int): Minimum pixel length the axes of an ellipse should have.
        _margin_inside (float): Maximum distance a point is allowed to be inside relative to the camera.
        _margin_outside (float): Maximum distance a point is allowed to be outside relative to the camera.
    """

    def __init__(self, left_or_right: str) -> None:
        """Constructor of the stone finder."""
        self._dimensions = (480, 640)
        self._lower_HSV_plate = np.array([0, 0, 77])
        self._upper_HSV_plate = np.array([57, 249, 228])
        self._lower_HSV_stone = np.array([62, 40, 113])
        self._upper_HSV_stone = np.array([174, 151, 219])
        self._decimation_filter = rs.decimation_filter(3)
        self._spatial_filter = rs.spatial_filter()
        self._align = rs.align(rs.stream.color)
        self._last_time_point_found = rospy.Time.now()
        self._not_found_counter = 0
        self._left_or_right = left_or_right

        self._retrieve_parameters()

        other_side = "right" if left_or_right == "left" else "left"
        self._other_frame_id = "toes_" + other_side + "_aligned"
        self._camera_frame_id = "camera_front_" + left_or_right + "_depth_optical_frame"

        self._listener = tf.TransformListener()
        self._point_publisher = rospy.Publisher("/march/foot_position/" + left_or_right, FootPosition, queue_size=1)
        self._marker_publisher = rospy.Publisher(
            "/camera_" + left_or_right + "/found_points", MarkerArray, queue_size=1
        )
        self._pointcloud_publisher = rospy.Publisher("/stone_finder_pointcloud", PointCloud2, queue_size=1)

    def _retrieve_parameters(self) -> None:
        """Retrieve parameters from the ros parameter server."""
        self._ellipse_similarity_threshold = rospy.get_param("~ellipse_similarity_threshold")
        self._minimum_ellipse_size = rospy.get_param("~minimum_ellipse_size")
        self._margin_inside = rospy.get_param("~margin_inside")
        self._margin_outside = rospy.get_param("~margin_outside")

    def find_points(self, frames: rs.composite_frame) -> None:
        """Find the closest ellipse center point in realsense color and depth frames.

        Args:
            frames (rs.composite_frame): A color and depth frame from a realsense pipeline.
        """
        self._retrieve_parameters()

        color_hsv_image, pointcloud = self._preprocess_frames(frames)
        color_segmented = self._color_segment(color_hsv_image)

        contours, _ = cv2.findContours(color_segmented, 1, 2)
        ellipses = self._find_ellipses(contours)
        closest_points = self._find_closest_points(ellipses, pointcloud)
        visualize_points = []
        point_published = False

        for point in closest_points:
            if point is not None and np.sum(np.abs(point)) > 0.02:  # check if point is not [0, 0, 0]

                if self._left_or_right == "left" and (
                    point[0] > self._margin_inside or point[0] < -self._margin_outside
                ):
                    continue
                if self._left_or_right == "right" and (
                    point[0] < -self._margin_inside or point[0] > self._margin_outside
                ):
                    continue

                try:
                    displacement = self._compute_displacement(point)
                    if not point_published:
                        publish_point(self._point_publisher, displacement)
                        point_published = True
                    visualize_points.append(displacement)

                    self._last_time_point_found = rospy.Time.now()
                    self._not_found_counter = 0
                    continue

                except (tf.LookupException, tf.ExtrapolationException, tf2.TransformException) as e:
                    rospy.logwarn(f"[march_stone_finder] Error {type(e).__name__} was raised.")

        publish_point_marker(self._marker_publisher, visualize_points)

        if rospy.Time.now() - self._last_time_point_found >= rospy.Duration(5.0):
            self._not_found_counter += 1
            self._last_time_point_found = rospy.Time.now()
            rospy.logwarn(
                f"[march_stone_finder] No stones found for {self._left_or_right} leg in last {self._not_found_counter * 5} seconds."
            )

    def _preprocess_frames(self, frames: rs.composite_frame) -> Tuple[np.ndarray, np.ndarray]:
        """Align depth and color frames, preprocess with realsense filters and a Gaussian blur,
        and generate and visualize a pointcloud.

        Args;
            frames (rs.composite_frame): A color and depth frame from a realsense pipeline.

        Returns:
            Tuple[np.ndarray, np.ndarray]: The preprocessed color image and the pointcloud.
        """
        filtered_frames = self._decimation_filter.process(frames).as_frameset()
        filtered_frames = self._spatial_filter.process(filtered_frames).as_frameset()
        aligned_frames = self._align.process(filtered_frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()

        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())

        color_image = np.asanyarray(color_frame.get_data())
        pointcloud = convert_depth_frame_to_pointcloud(
            depth_image, aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        )
        color_image = cv2.GaussianBlur(color_image, (3, 3), 0)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Visualize the pointcloud
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self._camera_frame_id
        pointcloud_reshaped = pointcloud.reshape((self._dimensions[0] * self._dimensions[1], 3))
        pointcloud_msg = pcl2.create_cloud_xyz32(header, pointcloud_reshaped[::8])
        self._pointcloud_publisher.publish(pointcloud_msg)

        return color_image, pointcloud

    def _color_segment(self, color_hsv_image: np.ndarray) -> np.ndarray:
        """Perform color segmentation on the images to select gray and remove brown.

        Args:
            color_hsv_image (np.ndarray): The color image in HSV color space.

        Returns:
            np.ndarray: The color segmented image in black and white.
        """
        mask_gray = cv2.inRange(color_hsv_image, self._lower_HSV_stone, self._upper_HSV_stone)
        mask_wood = cv2.inRange(color_hsv_image, self._lower_HSV_plate, self._upper_HSV_plate)
        white = np.full(self._dimensions, 255, np.uint8)
        filtered = cv2.bitwise_and(white, white, mask=mask_gray)
        return cv2.bitwise_and(filtered, cv2.bitwise_not(white, white, mask=mask_wood))

    def _find_ellipses(self, contours: List[np.ndarray]) -> List[cv2.ellipse]:
        """Find ellipse shapes from contours in a given image.

        Args:
            contours (List[np.ndarray]): A list of contours in the image.

        Returns:
            List[cv2.ellipse]: A list of ellipses in the image.s
        """
        ellipses = []
        for contour in contours:

            #  A contour requires at least 5 points to be able to fit an ellipse with cv2
            if len(contour) < 5:
                continue

            convex_hull = cv2.convexHull(contour)
            ellipse = cv2.fitEllipse(contour)

            try:
                contour_mask = np.zeros(self._dimensions, np.uint8)
                contour_mask = cv2.drawContours(contour_mask, convex_hull, -1, 255, 2)
                ellipse_mask = np.zeros(self._dimensions, np.uint8)
                ellipse_mask = cv2.ellipse(ellipse_mask, ellipse, 255, 2)
            except cv2.error:
                continue

            intersection = cv2.bitwise_and(contour_mask, ellipse_mask)

            num_intersection = cv2.countNonZero(intersection)
            num_contour = cv2.countNonZero(contour_mask)
            measure = num_intersection / num_contour
            bounds = ellipse[1]

            if (
                measure > self._ellipse_similarity_threshold
                and bounds[0] > self._minimum_ellipse_size
                and bounds[1] > self._minimum_ellipse_size
            ):
                ellipses.append(ellipse)
        return ellipses

    def _find_closest_points(self, ellipses: List[cv2.ellipse], pointcloud: np.ndarray) -> List[np.ndarray]:
        """Computes the distances of all ellipses to the camera, and returns the points sorted by distance in ascending order.

        Args:
            ellipses ([cv2.ellipse]): A list of found ellipses.
            pointcloud (np.ndarray): The source point cloud.

        Returns:
            List[np.ndarray]: Ellipse centers sorted by distance.
        """
        distances = []
        depth_points = []
        for ellipse in ellipses:
            centroid = ellipse[0]
            x_pixel = int(centroid[0])
            y_pixel = int(centroid[1])

            if y_pixel >= 0 and x_pixel >= 0 and y_pixel < pointcloud.shape[0] and x_pixel < pointcloud.shape[1]:
                depth_point = pointcloud[y_pixel][x_pixel]
                distances.append(np.linalg.norm(depth_point))
                depth_points.append(depth_point)

        return [point for (_, point) in sorted(zip(distances, depth_points), key=lambda pair: pair[0])]

    def _compute_displacement(self, point: np.ndarray) -> PointStamped:
        """Takes a found depth point as a numpy array, transforms it to the frame of the other leg and returns it as a PointStamped message.

        Args:
            point (np.ndarray): A depth point to step towards.

        Returns:
            PointStamped: The depth point in the other leg frame.
        """
        try:
            found_point = to_point_stamped(point)
            found_point.header.frame_id = self._camera_frame_id
            self._listener.waitForTransform(
                self._camera_frame_id, self._other_frame_id, rospy.Time.now(), rospy.Duration(1.0)
            )
            displacement = self._listener.transformPoint(self._other_frame_id, found_point)
            displacement.header.frame_id = self._other_frame_id

        except (tf.LookupException, tf.ExtrapolationException, tf2.TransformException) as e:
            raise e

        return displacement

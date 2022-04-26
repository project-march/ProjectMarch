"""Author: Tuhin Das, MVII"""

import rospy
import numpy as np
import pyrealsense2 as rs
from .utilities import convert_depth_frame_to_pointcloud, publish_point, to_point_stamped, publish_point_marker
import cv2
from march_shared_msgs.msg import FootPosition
import tf
from visualization_msgs.msg import Marker


class StoneFinder:
    def __init__(self, left_or_right):
        self.width = 640
        self.height = 480
        self.dimensions = (self.height, self.width)
        self.left_or_right = left_or_right
        self.initialized = False

        if left_or_right == "left":
            self.other_side = "right"
        else:
            self.other_side = "left"

        self.retrieve_parameters()

        self.current_frame_id = "toes_" + left_or_right + "_aligned"
        self.other_frame_id = "toes_" + self.other_side + "_aligned"
        self.camera_frame_id = "camera_front_left_depth_optical_frame"

        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher("/march_stone_finder/found_points/" + left_or_right, FootPosition, queue_size=1)
        self.marker_publisher = rospy.Publisher("/camera_" + left_or_right + "/found_points", Marker, queue_size=1)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def retrieve_parameters(self):
        self.minimum_connected_component_size = rospy.get_param("~minimum_connected_component_size")
        self.connectivity = rospy.get_param("~connectivity")
        self.ellipse_similarity_threshold = rospy.get_param("~ellipse_similarity_threshold")
        self.minimum_ellipse_size = rospy.get_param("~minimum_ellipse_size")

        self.lower_gray = np.array([0, 0, 168])
        self.upper_gray = np.array([172, 111, 255])
        self.lower_brown = np.array([1, 7, 110])
        self.upper_brown = np.array([55, 76, 206])

    def find_points(self, frames):
        self.retrieve_parameters()

        color_hsv_image, pointcloud = self.preprocess_frames(frames)
        color_segmented = self.color_segment(color_hsv_image)
        components = self.find_connected_components(color_segmented)

        components = cv2.morphologyEx(components, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        components = cv2.morphologyEx(components, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(components, 1, 2)
        ellipses = self.find_ellipses(contours)
        point = self.find_closest_point(ellipses, pointcloud)

        if point is not None and np.sum(np.abs(point)) > 0.02:
            try:
                displacement = self.compute_displacement(point)
                publish_point(self.publisher, displacement)
                # Visualize in rviz
                displacement.header.frame_id = self.other_frame_id
                self.listener.waitForTransform(self.other_frame_id, "world", rospy.Time.now(), rospy.Duration(0.100))
                displacement = self.listener.transformPoint("world", displacement)
                publish_point_marker(self.marker_publisher, displacement, "world")
            except Exception as e:
                print(e)

    def preprocess_frames(self, frames):
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()

        color_frame = aligned_frames.get_color_frame()
        depth_image = np.asanyarray(aligned_depth_frame.get_data())

        color_image = np.asanyarray(color_frame.get_data())
        pointcloud = convert_depth_frame_to_pointcloud(
            depth_image, aligned_depth_frame.profile.as_video_stream_profile().intrinsics
        )
        color_image = cv2.GaussianBlur(color_image, (5, 5), 0)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        return color_image, pointcloud

    def color_segment(self, color_hsv_image):
        mask_gray = cv2.inRange(color_hsv_image, self.lower_gray, self.upper_gray)
        mask_wood = cv2.inRange(color_hsv_image, self.lower_brown, self.upper_brown)
        white = np.full(self.dimensions, 255, np.uint8)
        filtered = cv2.bitwise_and(white, white, mask=mask_gray)
        return cv2.bitwise_and(filtered, cv2.bitwise_not(white, white, mask=mask_wood))

    def find_connected_components(self, color_segmented):
        result = np.full(self.dimensions, 0, np.uint8)
        n_components, output, _, _ = cv2.connectedComponentsWithStats(color_segmented, self.connectivity, cv2.CV_32S)

        for i in range(1, n_components + 1):
            pts = np.where(output == i)
            if len(pts[0]) >= self.minimum_connected_component_size:
                result[output == i] = 255

        return result

    def find_ellipses(self, contours):
        ellipses = []
        for contour in contours:
            if len(contour) < 5:
                continue

            convex_hull = cv2.convexHull(contour)
            ellipse = cv2.fitEllipse(contour)

            contour_mask = np.zeros(self.dimensions, np.uint8)
            contour_mask = cv2.drawContours(contour_mask, convex_hull, -1, (255, 255, 255), 2)

            ellipse_mask = np.zeros(self.dimensions, np.uint8)
            ellipse_mask = cv2.ellipse(ellipse_mask, ellipse, (255, 255, 255), 2)

            intersection = cv2.bitwise_and(contour_mask, ellipse_mask)

            num_intersection = cv2.countNonZero(intersection)
            num_contour = cv2.countNonZero(contour_mask)
            measure = num_intersection / num_contour
            bounds = ellipse[1]

            if (
                measure > self.ellipse_similarity_threshold
                and bounds[0] > self.minimum_ellipse_size
                and bounds[1] > self.minimum_ellipse_size
            ):
                ellipses += [ellipse]
        return ellipses

    def find_closest_point(self, ellipses, pointcloud):
        distances = []
        depthpoints = []
        for ellipse in ellipses:
            centroid = ellipse[0]
            x_pixel = int(centroid[0])
            y_pixel = int(centroid[1])

            if y_pixel < pointcloud.shape[0] and x_pixel < pointcloud.shape[1]:
                depthpoint = pointcloud[y_pixel][x_pixel]
                distances += [np.sqrt(np.sum(np.square(depthpoint)))]
                depthpoints += [depthpoint]

        if len(distances) > 0:
            return depthpoints[np.argmin(distances)]
        else:
            return None

    def compute_displacement(self, point):
        try:
            found_point = to_point_stamped(point)
            found_point.header.frame_id = self.camera_frame_id
            self.listener.waitForTransform(self.camera_frame_id, self.other_frame_id, rospy.Time.now(), rospy.Duration(0.100))
        except Exception as e:
            print(e)
        return self.listener.transformPoint(self.other_frame_id, found_point)

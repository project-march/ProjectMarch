"""Author: Tuhin Das, MVII."""

import numpy as np
import rospy
from rospy import Publisher
from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from typing import List
from pyrealsense2 import intrinsics


def convert_depth_frame_to_pointcloud(depth_image: np.ndarray, camera_intrinsics: intrinsics) -> np.ndarray:
    """Convert a realsense depth frame to a point cloud using the camera intrinsics.

    Args:
        depth_image (np.ndarray): Depth points from the depth camera.
        camera_intrinsics (intrinsics): Intrinsic details of the camera.

    Returns:
        np.ndarray: Pointcloud of the depth points.
    """
    [height, width] = depth_image.shape

    nx = np.linspace(0, width - 1, width)
    ny = np.linspace(0, height - 1, height)
    u, v = np.meshgrid(nx, ny)
    x = (u - camera_intrinsics.ppx) / camera_intrinsics.fx
    y = (v - camera_intrinsics.ppy) / camera_intrinsics.fy

    z = depth_image / 1000
    x = np.multiply(x, z)
    y = np.multiply(y, z)

    return np.dstack((x, y, z))


def publish_point(publisher: Publisher, point: PointStamped) -> None:
    """Publish a displacement which is used for dynamic gait generation.

    Args:
        publisher (Publisher): Publisher to publish the message with.
        point (PointStamped): Point to publish.
    """
    point_msg = FootPosition()
    point_msg.header.stamp = rospy.Time.now()
    point_msg.displacement.x = point.point.x
    point_msg.displacement.y = point.point.y
    point_msg.displacement.z = point.point.z
    publisher.publish(point_msg)


def publish_point_marker(publisher: Publisher, points: List[PointStamped]) -> None:
    """Publish a visualization marker array with all found ellipse centers.

    Args:
        publisher (Publisher): Publisher to publish the message with.
        point (List[PointStamped]): A list with PointStamped messages of ellipse circles.
    """
    marker_array = MarkerArray()

    for i, point_stamped in enumerate(points):
        marker = Marker()
        marker.header.frame_id = point_stamped.header.frame_id
        marker.header.stamp = rospy.get_rostime()
        marker.ns = "ellipse_center"
        marker.id = i
        marker.type = 1
        marker.action = 0

        marker.pose.position.x = point_stamped.point.x
        marker.pose.position.y = point_stamped.point.y
        marker.pose.position.z = point_stamped.point.z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        if "left" in point_stamped.header.frame_id and i == 0:
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
        elif "right" in point_stamped.header.frame_id and i == 0:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
        elif i > 0:
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0

        marker.color.a = 1.0
        marker.lifetime = rospy.Duration(0.3)

        marker_array.markers.append(marker)

    publisher.publish(marker_array)


def to_point_stamped(point: np.ndarray) -> PointStamped:
    """Convert a numpy point array to a PointStamped message.

    Args:
        point (np.ndarray): A point array of size (3,).

    Returns:
        PointStamped: A ros point message with a timestamp.
    """
    point_stamped = PointStamped()
    point_stamped.header.stamp = rospy.Time.now()
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]
    return point_stamped

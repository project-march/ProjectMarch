"""Author: Tuhin Das, MVII."""

import numpy as np
import rospy
from rospy import Publisher
from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from pyrealsense2 import intrinsics


def convert_depth_frame_to_pointcloud(depth_image: np.ndarray, camera_intrinsics: intrinsics) -> np.ndarray:
    """Convert a realsense depth frame to a point cloud using the camera intrinsics.

    Args:
        depth_image (np.ndarray): depth points from the depth camera
        camera_intrinsics (intrinsics): intrinsic details of the camera
    Returns:
        np.ndarray: pointcloud of the depth points
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


def publish_point(publisher: Publisher, point: np.ndarray) -> None:
    """Publish a displacement which is used for dynamic gait generation.

    Args:
        publisher (Publisher): publisher to publish the message with
        point (np.ndarray): point to publish
    """
    point_msg = FootPosition()
    point_msg.header.stamp = rospy.Time.now()
    point_msg.displacement.x = point.point.x
    point_msg.displacement.y = point.point.y
    point_msg.displacement.z = point.point.z
    publisher.publish(point_msg)


def publish_point_marker(publisher: Publisher, point: np.ndarray, frame: str) -> None:
    """Publish a visualization marker for the center of an ellipse.

    Args:
        publisher (Publisher): publisher to publish the message with
        point (np.ndarray): a point array of size (3,)
        frame (str): frame in which the point is published
    """
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "ellipse_center"
    marker.id = 10
    marker.type = 1
    marker.action = 0

    marker.pose.position.x = point.point.x
    marker.pose.position.y = point.point.y
    marker.pose.position.z = point.point.z
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02

    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.lifetime = rospy.Duration(0.3)

    publisher.publish(marker)


def to_point_stamped(point: np.ndarray) -> PointStamped:
    """Convert a numpy point array to a PointStamped message.

    Args:
        point (np.ndarray): a point array of size (3,)

    Returns:
        PointStamped: a ros point message with a timestamp
    """
    point_stamped = PointStamped()
    point_stamped.header.stamp = rospy.Time.now()
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]
    return point_stamped

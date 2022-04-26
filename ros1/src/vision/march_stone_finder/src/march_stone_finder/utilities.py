"""Author: Tuhin Das, MVII"""

import numpy as np
import rospy
from march_shared_msgs.msg import FootPosition
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


def convert_depth_frame_to_pointcloud(depth_image, camera_intrinsics):
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


def publish_point(publisher, point):
    point_msg = FootPosition()
    point_msg.header.stamp = rospy.Time.now()
    point_msg.displacement.x = point.point.x
    point_msg.displacement.y = point.point.y
    point_msg.displacement.z = point.point.z
    publisher.publish(point_msg)


def publish_point_marker(publisher, point, frame):
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


def to_point_stamped(point):
    point_stamped = PointStamped()
    point_stamped.header.stamp = rospy.Time.now()
    point_stamped.point.x = point[0]
    point_stamped.point.y = point[1]
    point_stamped.point.z = point[2]
    return point_stamped

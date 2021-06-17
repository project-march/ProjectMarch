#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

REPUBLISH_POSTFIX = "/sim_time"


def republish_cloud(publisher, msg):
    """
    Simple callback function to republish pointclouds.
    :param publisher: The publisher to republish on.
    :param msg: The message that should be republished
    """
    msg.header.stamp = rospy.Time.now()
    publisher.publish(msg)


rospy.init_node("republish_pointclouds", anonymous=True)
name = rospy.get_param("~name", "...")

CAMERA_TOPIC = f"/{name}/depth/color/points"
publisher = rospy.Publisher(
    CAMERA_TOPIC + REPUBLISH_POSTFIX, PointCloud2, queue_size=100
)
subscription = rospy.Subscriber(
    CAMERA_TOPIC,
    PointCloud2,
    lambda data: republish_cloud(publisher, data),
    queue_size=100,
)
rospy.spin()

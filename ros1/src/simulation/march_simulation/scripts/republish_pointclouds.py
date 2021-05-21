#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2

rospy.init_node('republish_pointclouds', anonymous=True)

CAMERA_FRONT_TOPIC = '/camera_front/depth/color/points'
CAMERA_BACK_TOPIC = '/camera_back/depth/color/points'
REPUBLISH_POSTFIX = "/sim_time"

for topic in (CAMERA_BACK_TOPIC, CAMERA_FRONT_TOPIC):
    publisher = rospy.Publisher(topic + REPUBLISH_POSTFIX, PointCloud2)
    subscription = rospy.Subscriber(
        topic,
        PointCloud2,
        lambda data: republish_cloud(publisher, data)
    )


def republish_cloud(publisher, msg):
    """
    Simple callback function to republish pointclouds.
    :param publisher: The publisher to republish on.
    :param msg: The message that should be republished
    """
    msg.header.stamp = rospy.Time(0)
    publisher.publish(msg)


rospy.spin()
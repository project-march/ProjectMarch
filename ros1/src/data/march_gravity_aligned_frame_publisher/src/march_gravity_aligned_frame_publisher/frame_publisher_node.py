"""Author: Jelmer de Wolde, MVII."""

import rospy
import tf

# Define the distance from frame 'foot_left' or 'foot_right' at which the foot rotates:
FOOT_LENGTH = 0.2
Z_ROTATION = [0.0, 0.0, 0.0, 1.0]
RATE = 10.0


def main():
    """A node that publishes two frames around toes for both feet.

    The first frame is a transformation from the 'foot_left'/'foot_right' to the toes, rotated 180 degrees around
    the z-axis. The second frame is te first described frame, but with the z-axis aligned with the gravitation force.
    """
    rospy.init_node("frame_aligner")

    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        try:
            (trans_left, rot_left) = listener.lookupTransform("/foot_left", "/world", rospy.Time(0))
            (trans_right, rot_right) = listener.lookupTransform("/foot_right", "/world", rospy.Time(0))
            (trans_hip, rot_hip) = listener.lookupTransform("/hip_base", "/world", rospy.Time(0))
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            continue

        broadcaster.sendTransform(
            (-FOOT_LENGTH, 0.0, -0.01),
            Z_ROTATION,
            rospy.Time.now(),
            "toes_left",
            "foot_left",
        )

        broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            rot_left,
            rospy.Time.now(),
            "toes_left_aligned",
            "toes_left",
        )

        broadcaster.sendTransform(
            (-FOOT_LENGTH, 0.0, -0.01),
            Z_ROTATION,
            rospy.Time.now(),
            "toes_right",
            "foot_right",
        )

        broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            rot_right,
            rospy.Time.now(),
            "toes_right_aligned",
            "toes_right",
        )

        broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            rot_hip,
            rospy.Time.now(),
            "hip_base_aligned",
            "hip_base",
        )

        rate.sleep()

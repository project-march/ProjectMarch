"""Executable to start the moveit interface node."""
import rospy


def main():
    rospy.init_node("balance_gaits")

    rospy.spin()

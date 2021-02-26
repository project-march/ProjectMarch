"""Executable to start the moveit interface node."""
import rospy
from march_shared_msgs.srv import (
    GetMoveItTrajectory,
)
from march_moveit_interface.moveit_interface import MoveItInterface


def main():
    rospy.init_node("balance_gaits")

    moveit_interface = MoveItInterface()

    rospy.Service(
        "/march/moveit/get_trajectory",
        GetMoveItTrajectory,
        moveit_interface.get_joint_trajectory,
    )

    rospy.spin()

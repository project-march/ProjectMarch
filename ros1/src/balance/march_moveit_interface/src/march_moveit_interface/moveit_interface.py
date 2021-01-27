from copy import deepcopy
import os
import sys
from typing import Optional

import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from march_shared_msgs.srv import CapturePointPose, GetMoveItTrajectoryRequest, \
    GetMoveItTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory


class MoveItInterface:
    """Base class to create a gait using the moveit motion planning."""

    def __init__(self):

        self._end_effectors = {"left_leg": "foot_left", "right_leg": "foot_right"}

        self.move_group = {}

        self._joint_state_target = JointState()
        self._joint_state_target.header = Header()
        self._joint_state_target.name = ""

        moveit_commander.roscpp_initialize(sys.argv)
        moveit_commander.RobotCommander()

        moveit_commander.PlanningSceneInterface()

        try:
            self.move_group = {
                "all_legs": moveit_commander.MoveGroupCommander("all_legs"),
                "left_leg": moveit_commander.MoveGroupCommander("left_leg"),
                "right_leg": moveit_commander.MoveGroupCommander("right_leg"),
            }

            for move_group in self.move_group.values():
                move_group.set_pose_reference_frame("world")

        except RuntimeError:
            rospy.logerr(
                "Could not connect to move groups, aborting initialisation of the "
                "moveit interface node"
            )

    def set_swing_leg_target(self, leg_name: str, target_pose: Pose):
        """Set the swing leg target to capture point.

        :param leg_name: The name of the used move group
        :param subgait_name: the normal subgait name
        """
        self.move_group[leg_name].set_joint_value_target(
            target_pose, self._end_effectors[leg_name], True
        )

    def set_stance_leg_target(self, leg_name, joint_state):
        """Set the target of the stance leg to the end of the gait file.

        :param leg_name: The name of the move group which does not use capture point
        :param subgait_name: the normal subgait name

        :return the duration of the default subgait
        """
        self.move_group[leg_name].set_joint_value_target(joint_state)

    def construct_trajectory(self, swing_leg: str, swing_leg_target: Pose,
                             stance_leg_target: JointState) -> Optional[
        JointTrajectory]:
        """Constructs a balance trajectory for all joints.

        :param capture_point_leg_name: The name of the move group that should be used to create the balance subgait
        :param subgait_name: the normal subgait name

        :return: the balance trajectory
        """
        rospy.loginfo("Constructing trajectory")
        stance_leg = "right_leg" if swing_leg == "left_leg" else "left_leg"

        self.set_swing_leg_target(swing_leg, swing_leg_target)
        rospy.loginfo("Done setting swing leg target")
        self.set_stance_leg_target(stance_leg, stance_leg_target)
        rospy.loginfo("Done setting stance leg target")

        self._joint_state_target = JointState()

        targets = (
            self.move_group["left_leg"].get_joint_value_target()
            + self.move_group["right_leg"].get_joint_value_target()
        )

        self._joint_state_target.header.stamp = rospy.Time.now()
        self._joint_state_target.position = targets

        rospy.loginfo("Planning")
        plan = self.move_group["all_legs"].plan(
            self._joint_state_target)
        rospy.loginfo("Done planning")

        if not plan[0]:
            rospy.logwarn(
                f"No valid balance trajectory for {swing_leg} received from capture "
                f"point leg, returning default subgait"
            )
            return None

        return plan[1]

    def get_joint_trajectory(self, req: GetMoveItTrajectoryRequest) -> GetMoveItTrajectoryResponse:
        """Returns the trajectory of a subgait name that could use moveit.

        :param name: the name of the subgait
        """
        res = GetMoveItTrajectoryResponse()
        trajectory = self.construct_trajectory(req.swing_leg, req.swing_leg_target_pose,
                                                   req.stance_leg_target)
        if trajectory is not None:
            res.success = True
            res.trajectory = trajectory
        else:
            res.success = False
        return res

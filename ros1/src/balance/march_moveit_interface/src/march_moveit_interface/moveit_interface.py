from copy import deepcopy
import os
import sys
from typing import Optional

import moveit_commander
import rospy
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

        self.trajectory_publisher = rospy.Publisher("/traj", JointTrajectory, 10)
        self.log_publisher = rospy.Publisher('/log_test', String, 10)

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

    def set_swing_leg_target(self, leg_name, joint_state):
        """Set the swing leg target to capture point.

        :param leg_name: The name of the used move group
        :param subgait_name: the normal subgait name
        """
        self.move_group[leg_name].set_joint_value_target(
            joint_state, self._end_effectors[leg_name], True
        )

    def set_stance_leg_target(self, leg_name, joint_state):
        """Set the target of the stance leg to the end of the gait file.

        :param leg_name: The name of the move group which does not use capture point
        :param subgait_name: the normal subgait name

        :return the duration of the default subgait
        """
        self.move_group[leg_name].set_joint_value_target(joint_state)

    def construct_trajectory(self, swing_leg: str, swing_leg_target: JointState,
                             stance_leg_target: JointState) -> Optional[
        JointTrajectory]:
        """Constructs a balance trajectory for all joints.

        :param capture_point_leg_name: The name of the move group that should be used to create the balance subgait
        :param subgait_name: the normal subgait name

        :return: the balance trajectory
        """
        stance_leg = "right_leg" if swing_leg == "left_leg" else "left_leg"

        self.set_swing_leg_target(swing_leg, swing_leg_target)
        self.set_stance_leg_target(stance_leg, stance_leg_target)

        targets = (
            self.move_group["left_leg"].get_joint_value_target()
            + self.move_group["right_leg"].get_joint_value_target()
        )

        self._joint_state_target.header.stamp = rospy.Time.now()
        self._joint_state_target.position = targets

        balance_subgait = self.move_group["all_legs"].plan(self._joint_state_target)
        balance_trajectory = balance_subgait.joint_trajectory

        if not balance_trajectory:
            rospy.logwarn(
                "No valid balance trajectory for {ln} received from capture point leg, "
                "returning default subgait".format(ln=swing_leg)
            )
            return None

        if not balance_trajectory.points:
            rospy.logwarn(
                "Empty trajectory in {ln} received from capture point topic, "
                "returning default subgait".format(ln=swing_leg)
            )
            return None

        self.trajectory_publisher.publish(balance_trajectory)

        return balance_trajectory

    def get_joint_trajectory(self, req: GetMoveItTrajectoryRequest,
                                 res: GetMoveItTrajectoryResponse) -> None:
        """Returns the trajectory of a subgait name that could use moveit.

        :param name: the name of the subgait
        """

        self.log_publisher.publish('Request received')
        trajectory = self.construct_trajectory(req.swing_leg, req.swing_leg_target,
                                                   req.stance_leg_target)
        self.log_publisher.publish('Trajectory constructed')
        self.trajectory_publisher.publish(trajectory)
        if trajectory is not None:
            res.succes = True
            res.trajectory = trajectory
        else:
            res.succes = False
        return

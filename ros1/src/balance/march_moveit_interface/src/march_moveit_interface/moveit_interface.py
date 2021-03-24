"""
Contains a moveit interface class for planning trajectories.

This class is used to allow ros2 gait selection to plan trajectories when the balance
gait is being executed.
"""
import sys
from typing import Optional

import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from march_shared_msgs.srv import (
    GetMoveItTrajectoryRequest,
    GetMoveItTrajectoryResponse,
)
from trajectory_msgs.msg import JointTrajectory


class MoveItInterface:
    """Base class to create a gait using the moveit motion planning."""

    def __init__(self):
        self._end_effectors = {"left_leg": "foot_left", "right_leg": "foot_right"}

        self.move_group = {}
        self._joint_state_target = JointState()
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

        :param leg_name: The name of the used move group.
        :param target_pose: The capture point pose that the swing leg should go to.
        """
        # The move group sets a target for moveit, the end effector should be
        # specified, since otherwise the default is used, the boolean (True)
        # specifies that the pose is an approximate.
        self.move_group[leg_name].set_joint_value_target(
            target_pose, self._end_effectors[leg_name], True
        )

    def set_stance_leg_target(self, leg_name: str, joint_state: JointState):
        """Set the target of the stance leg to the end of the gait file.

        :param leg_name: The name of the move group which does not use capture point.
        :param joint_state: The end state in which the stance leg should be planned.
        """
        self.move_group[leg_name].set_joint_value_target(joint_state)

    def construct_trajectory(
        self, swing_leg: str, swing_leg_target: Pose, stance_leg_target: JointState
    ) -> Optional[JointTrajectory]:
        """Construct a balance trajectory for all joints.

        :param swing_leg: The name of the swing leg.
        :param swing_leg_target: The capture point target for swing leg.
        :param stance_leg_target: The target state for the stance leg.

        :return: The balance trajectory
        """
        stance_leg = "right_leg" if swing_leg == "left_leg" else "left_leg"

        self.set_swing_leg_target(swing_leg, swing_leg_target)
        self.set_stance_leg_target(stance_leg, stance_leg_target)

        self._joint_state_target = JointState()

        targets = (
            self.move_group["left_leg"].get_joint_value_target()
            + self.move_group["right_leg"].get_joint_value_target()
        )

        self._joint_state_target.header.stamp = rospy.Time.now()
        self._joint_state_target.position = targets

        plan = self.move_group["all_legs"].plan(self._joint_state_target)

        if not plan[0]:
            rospy.logwarn(
                f"No valid balance trajectory for {swing_leg} received from capture "
                f"point leg, returning default subgait."
            )
            return None

        return plan[1].joint_trajectory

    def get_joint_trajectory(
        self, req: GetMoveItTrajectoryRequest
    ) -> GetMoveItTrajectoryResponse:
        """Create a moveit trajectory as a service callback.

        :param req: The request for the moveit trajectory.
        """
        res = GetMoveItTrajectoryResponse()
        if req.swing_leg not in ["right_leg", "left_leg"]:
            res.success = False
            rospy.logwarn(
                f"Incorrect swing leg {req.swing_leg} was given, no moveit "
                f"trajectory can be constructed."
            )
            return res
        trajectory = self.construct_trajectory(
            req.swing_leg, req.swing_leg_target_pose, req.stance_leg_target
        )
        if trajectory is not None:
            res.success = True
            res.trajectory = trajectory
        else:
            res.success = False
        return res

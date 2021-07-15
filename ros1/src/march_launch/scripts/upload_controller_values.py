#!/usr/bin/env python3
from typing import List

import rospy
import sys
from urdf_parser_py import urdf

CONTROLLER_PREFIX = "/march/controller"


def get_controller_names(controller_type: str) -> List[str]:
    """Get active controller names

    Returns an extra controller name if the controller type is mixed control.
    """
    controller_names = ["trajectory"]
    if controller_type == "mixed_control":
        controller_names.append("trajectory_mpc")
    return controller_names


def get_follow_joint_trajectory_bridge_topics(controller_name: str) -> List[dict]:
    """Get all bridge topics for a certain controller."""
    return [
        {
            "topic": f"{CONTROLLER_PREFIX}/{controller_name}/follow_joint_trajectory/cancel",
            "type": "actionlib_msgs/msg/GoalID",
        },
        {
            "topic": f"{CONTROLLER_PREFIX}/{controller_name}/follow_joint_trajectory/goal",
            "type": "march_shared_msgs/msg/FollowJointTrajectoryActionGoal",
        },
        {
            "topic": f"{CONTROLLER_PREFIX}/{controller_name}/follow_joint_trajectory/feedback",
            "type": "march_shared_msgs/msg/FollowJointTrajectoryActionFeedback",
        },
        {
            "topic": f"{CONTROLLER_PREFIX}/{controller_name}/follow_joint_trajectory/result",
            "type": "march_shared_msgs/msg/FollowJointTrajectoryActionResult",
        },
        {
            "topic": f"{CONTROLLER_PREFIX}/{controller_name}/follow_joint_trajectory/status",
            "type": "actionlib_msgs/msg/GoalStatusArray",
        },
        {
            "topic": f"{CONTROLLER_PREFIX}/{controller_name}/trajectory/state",
            "type": "control_msgs/msg/JointTrajectoryControllerState",
        },
        {
            "topic": f"{CONTROLLER_PREFIX}/{controller_name}/command",
            "type": "trajectory_msgs/msg/JointTrajectory",
        },
    ]


def get_params_for_actuation(joint: str, controller_name: str) -> List[str]:
    """
    Give all parameters that are set for every joint to be able to spawn a controller
    :param joint: The name of the joint
    :return: The parameter names
    """
    return [
        f"{CONTROLLER_PREFIX}/{controller_name}/gains/{joint}",
        f"{CONTROLLER_PREFIX}/{controller_name}/constraints/{joint}",
    ]


def get_actuating_joint_names_by_controller(
    controller_name: str, actuating_joint_names: List[str]
) -> List[str]:
    """
    Compare the actuating joint names with the joint corresponding to a certain controller.
    :param controller_name: Controller to look up joint names for
    :param actuating_joint_names: All actuating joint names
    :return: Intersection of the actuating joint names and the controller joint names
    """
    joint_names_by_controller = rospy.get_param(
        f"/march/controller/{controller_name}/joints"
    )
    actuating_joint_names_by_controller = set(actuating_joint_names).intersection(
        set(joint_names_by_controller)
    )
    return sorted(actuating_joint_names_by_controller)


def main():
    """
    This script looks at the values that were uploaded for the controller and removes
    values for joints that are fixed in the urdf. By calling this before spawning the
    controller it is made easier to change what joints we are currently working with
    and only controlling these. This way, there are not different config files required
    for every combination of joints.
    """
    try:
        rospy.init_node("upload_controller_values")
    except rospy.ROSInitException:
        return

    current_bridge_topics = rospy.get_param("/topics")
    controller_names = get_controller_names(sys.argv[1])
    for controller_name in controller_names:
        for bridge_topic in get_follow_joint_trajectory_bridge_topics(controller_name):
            current_bridge_topics.append(bridge_topic)
    rospy.set_param("/topics", current_bridge_topics)

    robot = urdf.Robot.from_parameter_server("/robot_description")
    actuating_joint_names = []
    fixed_joint_names = []

    for joint in robot.joints:
        if joint.type != "fixed":
            actuating_joint_names.append(joint.name)
        else:
            fixed_joint_names.append(joint.name)

    actuating_joint_names = sorted(actuating_joint_names)
    if len(actuating_joint_names) == 0:
        rospy.logerr("No actuating joints were specified.")

    if not rospy.is_shutdown():
        rospy.set_param("/march/joint_names", actuating_joint_names)

        for controller_name in controller_names:
            actuating_joint_names_by_controller = (
                get_actuating_joint_names_by_controller(
                    controller_name, actuating_joint_names
                )
            )
            rospy.set_param(
                f"/march/controller/{controller_name}/joints",
                actuating_joint_names_by_controller,
            )

            for joint in fixed_joint_names:
                for param in get_params_for_actuation(joint, controller_name):
                    try:
                        rospy.delete_param(param)
                    except KeyError:
                        continue


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

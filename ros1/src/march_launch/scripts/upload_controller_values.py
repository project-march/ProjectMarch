#! /usr/bin/env python3
from typing import List

import rospy
from urdf_parser_py import urdf

def get_params_for_actuation(joint: str) -> List[str]:
    """
    Give all parameters that are set for every joint to be able to spawn a controller
    :param joint: The name of the joint
    :return: The parameter names
    """
    return [
        f"/march/controller/trajectory/gains/{joint}",
        f"/march/controller/trajectory/constraints/{joint}",
    ]


def main():
    """
    This script looks at the values that were uploaded for the controller and removes
    values for joints that are fixed in the urdf. By calling this before spawning the
    controller it is made easier to change what joints we are currently working with
    and only controlling these. This way, there are not different config files required
    for every combination of joints.
    """
    rospy.init_node("upload_controller_values")
    robot = urdf.Robot.from_parameter_server("/robot_description")
    actuating_joint_names = []
    fixed_joint_names = []
    for joint in robot.joints:
        if joint.type != "fixed":
            actuating_joint_names.append(joint.name)
        else:
            fixed_joint_names.append(joint.name)
    rospy.set_param("/march/joint_names", actuating_joint_names)
    rospy.set_param("/march/controller/trajectory/joints", actuating_joint_names)

    for joint in fixed_joint_names:
        for param in get_params_for_actuation(joint):
            try:
                rospy.delete_param(param)
            except KeyError:
                continue


if __name__ == "__main__":
    main()

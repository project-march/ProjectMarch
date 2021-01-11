import os

import rospkg
from urdf_parser_py import urdf

from march_shared_classes.exceptions.general_exceptions import SideSpecificationError

from .side import Side


def weighted_average(base_value, other_value, parameter):
    """Compute the weighted average of two values with normalised weight parameter.

    :param base_value: The first value for the weighted average, return this if parameter is 0
    :param other_value: The second value for the weighted average, return this if parameter is 1
    :param: parameter: The normalised weight parameter, the parameter that deterines the weight of the second value

    :return: A value which is the weighted average of the given values
    """
    return base_value * (1 - parameter) + other_value * parameter


def merge_dictionaries(dic_one, dic_two):
    """Combines the key value pairs of two dicitonaries into a new dicionary.

    Throws an error when both dictionaries contain the same key and the corresponding values are not equal.

    :param dic_one: One of the dictionaries which is to be merged
    :param dic_two: One of the dictionaries which is to be merged

    :return: The merged dictionary, has the same key value pairs as the pairs in the given dictionaries combined
    """
    merged_dic = {}
    for key_one in dic_one:
        if key_one not in dic_two or dic_one[key_one] == dic_two[key_one]:
            merged_dic[key_one] = dic_one[key_one]
        else:
            raise KeyError(
                "Dictionaries to be merged both contain key {key} with differing values".format(
                    key=key_one
                )
            )
    for key_two in dic_two:
        if key_two not in dic_one or dic_one[key_two] == dic_two[key_two]:
            merged_dic[key_two] = dic_two[key_two]
        else:
            raise KeyError(
                "Dictionaries to be merged both contain key {key} with differing values".format(
                    key=key_two
                )
            )
    return merged_dic


def get_lengths_robot_for_inverse_kinematics(side=None):
    """Grabs lengths from the robot which are relevant for the inverse kinematics calculation.

    this function returns the lengths relevant for the specified foot, if no side is specified,
    it returns all relevant lengths for both feet.

    :param side: The side of the exoskeleton of which the relevant would like to be known
    :return: The lengths of the specified side which are relevant for the (inverse) kinematics calculations
    """
    try:
        robot = urdf.Robot.from_xml_file(
            os.path.join(
                rospkg.RosPack().get_path("march_description"), "urdf", "march4.urdf"
            )
        )
        # size[0], size[1] and size[2] are used to grab relevant length of the link, e.g. the relevant length of the
        # hip base is in the y direction, that of the upper leg in the z direction.
        base = (
            robot.link_map["hip_base"].collisions[0].geometry.size[1]
        )  # length of the hip base structure
        l_ul = (
            robot.link_map["upper_leg_left"].collisions[0].geometry.size[2]
        )  # left upper leg length
        l_ll = (
            robot.link_map["lower_leg_left"].collisions[0].geometry.size[2]
        )  # left lower leg length
        l_hl = (
            robot.link_map["hip_aa_frame_left_front"].collisions[0].geometry.size[0]
        )  # left haa arm to leg
        l_ph = (
            robot.link_map["hip_aa_frame_left_side"].collisions[0].geometry.size[1]
        )  # left pelvis to hip length
        r_ul = (
            robot.link_map["upper_leg_right"].collisions[0].geometry.size[2]
        )  # right upper leg length
        r_ll = (
            robot.link_map["lower_leg_right"].collisions[0].geometry.size[2]
        )  # right lower leg length
        r_hl = (
            robot.link_map["hip_aa_frame_right_front"].collisions[0].geometry.size[0]
        )  # right haa arm to leg
        r_ph = (
            robot.link_map["hip_aa_frame_right_side"].collisions[0].geometry.size[1]
        )  # right pelvis hip length
        # the foot is a certain amount more to the inside of the exo then the leg structures.
        # The haa arms need to account for this.
        off_set = (
            robot.link_map["ankle_plate_right"].visual.origin.xyz[1] + base / 2 + r_hl
        )
        r_ph = r_ph - off_set
        l_ph = l_ph - off_set

    except KeyError as e:
        raise KeyError(
            'Expected robot.link_map to contain "{key}", but "{key}" was missing.'.format(
                key=e.args[0]
            )
        )

    if side == Side.left:
        return [l_ul, l_ll, l_hl, l_ph, base]
    elif side == Side.right:
        return [r_ul, r_ll, r_hl, r_ph, base]
    elif side == Side.both:
        return [l_ul, l_ll, l_hl, l_ph, r_ul, r_ll, r_hl, r_ph, base]
    else:
        raise SideSpecificationError(
            side,
            "Side should be either 'left', 'right' or 'both', but was {side}".format(
                side=side
            ),
        )


def get_joint_names_for_inverse_kinematics():
    robot = urdf.Robot.from_xml_file(
        os.path.join(
            rospkg.RosPack().get_path("march_description"), "urdf", "march4.urdf"
        )
    )
    robot_joint_names = robot.joint_map.keys()
    joint_name_list = [
        "left_hip_aa",
        "left_hip_fe",
        "left_knee",
        "right_hip_aa",
        "right_hip_fe",
        "right_knee",
    ]
    for joint_name in joint_name_list:
        if joint_name not in robot_joint_names:
            raise KeyError(
                "Inverse kinematics calculation expected the robot to have joint "
                "{joint_name}, but {joint_name} was not found.".format(
                    joint_name=joint_name
                )
            )

    return joint_name_list

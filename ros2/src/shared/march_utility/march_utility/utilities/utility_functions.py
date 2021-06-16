"""This module contains general helper functions.

These functions are not a part of any specific part of the code, but will be useful
for general use cases.
"""

import os
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from urdf_parser_py import urdf
from rclpy.node import Node

from march_utility.exceptions.general_exceptions import SideSpecificationError
from march_utility.utilities.vector_3d import Vector3d
from .side import Side

import yaml


def weighted_average_floats(
    base_value: float, other_value: float, parameter: float
) -> float:
    """
    Compute the weighted average of two element with normalised weight parameter.

    :param base_value: The first value for the weighted average,
        return this if parameter is 0
    :param other_value: The second value for the weighted average,
        return this if parameter is 1
    :param parameter: The normalised weight parameter, the parameter that
        determines the weight of the second value

    :return: A vector which is the weighted average of the given values
    """
    return base_value * (1 - parameter) + other_value * parameter


def weighted_average_vectors(
    base_vector: Vector3d, other_vector: Vector3d, parameter: float
) -> Vector3d:
    """
    Compute the weighted average of two element with normalised weight parameter.

    :param base_vector: The first vector for the weighted average,
        return this if parameter is 0
    :param other_vector: The second vector for the weighted average,
        return this if parameter is 1
    :param parameter: The normalised weight parameter, the parameter that
        determines the weight of the second vector

    :return: A vector which is the weighted average of the given vectors
    """
    return base_vector * (1 - parameter) + other_vector * parameter


def merge_dictionaries(dic_one: dict, dic_two: dict) -> dict:
    """Combine the key value pairs of two dicitonaries into a new dictionary.

    Throws an error when both dictionaries contain the same key and the
    corresponding values are not equal.
    :param dic_one: One of the dictionaries which is to be merged
    :param dic_two: One of the dictionaries which is to be merged
    :return: The merged dictionary, has the same key value pairs as the pairs in
    the given dictionaries combined
    """
    merged_dic = {}
    for key_one in dic_one:
        if check_key(dic_one, dic_two, key_one):
            merged_dic[key_one] = dic_one[key_one]
    for key_two in dic_two:
        if check_key(dic_one, dic_two, key_two):
            merged_dic[key_two] = dic_two[key_two]

    return merged_dic


def check_key(dic_one: dict, dic_two: dict, key: str) -> bool:
    """Check whether this key can be merged between two dictionaries."""
    if key not in dic_one or key not in dic_two:
        return True
    else:
        if dic_one[key] != dic_two[key]:
            raise KeyError(
                f"Dictionaries to be merged both contain key {key} with differing "
                f"values"
            )
        return True


def select_lengths_for_inverse_kinematics(
    lengths: List[float], side: Side = Side.both
) -> List[float]:
    """Return only the lengths in the list on the requested side."""
    if len(lengths) != 9:
        Node("march_utility").get_logger().error(
            "The lengths given did not have size 9. Cannot unpack the lengths."
        )

    l_ul, l_ll, l_hl, l_ph, r_ul, r_ll, r_hl, r_ph, base = lengths
    if side == Side.left:
        return [l_ul, l_ll, l_hl, l_ph, base]
    elif side == Side.right:
        return [r_ul, r_ll, r_hl, r_ph, base]
    return lengths


def get_lengths_robot_from_urdf_for_inverse_kinematics(  # noqa: CCR001
    side: Side = Side.both,
) -> List[float]:
    """Grab lengths which are relevant for the inverse kinematics calculation from the urdf file.

    This function returns the lengths relevant for the specified foot, if no
    side is specified,it returns all relevant lengths for both feet.

    :param side: The side of the exoskeleton of which the lengths would like to be known
    :return: The lengths of the specified side which are relevant for
        the (inverse) kinematics calculations
    """
    if not isinstance(side, Side):
        raise SideSpecificationError(
            side, f"Side should be either 'left', 'right' or 'both', but was {side}"
        )
    try:
        yaml_file = open(
            os.path.join(
                get_package_share_directory("march_description"),
                "urdf",
                "properties",
                "march6.yaml",
            ),
            "r",
        )

        robot_dimensions = yaml.safe_load(yaml_file)["dimensions"]

        base_length = robot_dimensions["hip_base"]["length"]
        hip_side_length = robot_dimensions["hip_aa_side"]["length"]
        hip_front_length = robot_dimensions["hip_aa_front"]["length"]
        upper_leg_length = robot_dimensions["upper_leg"]["length"]
        lower_leg_length = robot_dimensions["lower_leg"]["length"]
        ankle_offset = (
            robot_dimensions["upper_leg"]["offset"]
            + robot_dimensions["ankle_plate"]["offset"]
        )
        hip_aa_arm_length = hip_side_length - ankle_offset

    except KeyError as e:
        raise KeyError(
            f"Expected robot.link_map to contain {e.args[0]}, but it was missing."
        )

    return select_lengths_for_inverse_kinematics(
        [
            upper_leg_length,
            lower_leg_length,
            hip_front_length,
            hip_aa_arm_length,
            upper_leg_length,
            lower_leg_length,
            hip_front_length,
            hip_aa_arm_length,
            base_length,
        ],
        side,
    )


LENGTHS_BOTH_SIDES = get_lengths_robot_from_urdf_for_inverse_kinematics()


def get_lengths_robot_for_inverse_kinematics(side: Side = Side.both) -> List[float]:
    """Grab lengths which are relevant for the inverse kinematics calculations from a list."""
    return select_lengths_for_inverse_kinematics(LENGTHS_BOTH_SIDES, side)


def validate_and_get_joint_names_for_inverse_kinematics(logger=None)\
        -> \
        Optional[
    List[
    str]]:
    """Get a list of the joint names that can be used for the inverse kinematics.

    Returns none if the robot description does not contain the required joints.
    :return: A list of joint names.
    """
    robot = urdf.Robot.from_xml_file(
        os.path.join(
            get_package_share_directory("march_description"), "urdf", "march6.urdf"
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
        if (
            joint_name not in robot_joint_names
            or robot.joint_map[joint_name].type == "fixed"
        ):
            if logger is not None:
                logger.warn(f"{joint_name} is fixed, but required for IK")
            return None

    return sorted(joint_name_list)
